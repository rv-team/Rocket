// Author：Mio  2019/7/24  V1.1
// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket

import Chisel._
import Chisel.ImplicitConversions._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.{DescribedSRAM, _}
import freechips.rocketchip.util.property._
import chisel3.internal.sourceinfo.SourceInfo
import chisel3.experimental.dontTouch
import freechips.rocketchip.diplomaticobjectmodel.DiplomaticObjectModelAddressing
import freechips.rocketchip.diplomaticobjectmodel.model._

/*
 *ICacheParams：定义iCache中所需参数；
 *4-way set cache; 共64组，每组数据大小=64B；
 *每行所占据的字节数为 128b=4字，即每一路占据1字；
 *tagECC/dataECC: Error Correcting Code for tag/data part,大致意思是检查并纠错tag/data中数据，但为什么这么设置仍待探索;
 *itimAddr(instruction tightly intergate memory)：The instruction cache can be partially reconfigured into ITIM, which occupies a fixed address
range in the memory map.Fetching an instruction from ITIM is as fast as an instruction-cache hit, with no possibility of a
cache miss.
 *latency：读取cache所需2 cycles；
 *fetchBytes：取一条指令需要4B=1字=32b；
 *tagCode/dataCode：配置cache中的Tag和Data；
 *replacement：随机替换作为替换策略；
*/
case class ICacheParams(
    nSets: Int = 64,
    nWays: Int = 4,
    rowBits: Int = 128,
    nTLBEntries: Int = 32,
    cacheIdBits: Int = 0,
    tagECC: Option[String] = None,  
    dataECC: Option[String] = None,
    itimAddr: Option[BigInt] = None,  //？
    prefetch: Boolean = false,
    blockBytes: Int = 64,
    latency: Int = 2,
    fetchBytes: Int = 4) extends L1CacheParams {
  def tagCode: Code = Code.fromString(tagECC)
  def dataCode: Code = Code.fromString(dataECC)
  def replacement = new RandomReplacement(nWays)
}

/*HasL1ICacheParameters：定义L1-I$的参数；
 *具体定义方法是沿用了HasCoreParameters中tileParams.icache.get所定义的，归根结底就是引用了ICacheParams；
 *I$的互联方式采用的是tilelink（由UCBer提出）：TileLink is a chip-scale interconnect standard providing multiple masters with coherent memorymapped access to memory and other slave devices. 
*/
trait HasL1ICacheParameters extends HasL1CacheParameters with HasCoreParameters {
  val cacheParams = tileParams.icache.get
}

/*ICacheReq：ICache接受到的请求信息；
 *addr：PC所指向的虚拟地址，位宽=vaddrBits；
*/
class ICacheReq(implicit p: Parameters) extends CoreBundle()(p) with HasL1ICacheParameters {
  val addr = UInt(width = vaddrBits)
}

/*ICacheErrors：定义ICache经过ECC后错误是否可纠正；
 *correctable(width = paddrBits)：标记该错误可被纠正<- tag/data error candetect=1;
 *uncorrectable:：标记该错误不可被纠正<-itimAddr不为空且datacode可被检测;
*/
class ICacheErrors(implicit p: Parameters) extends CoreBundle()(p)
    with HasL1ICacheParameters
    with CanHaveErrors {
  val correctable = (cacheParams.tagCode.canDetect || cacheParams.dataCode.canDetect).option(Valid(UInt(width = paddrBits)))   //？
  val uncorrectable = (cacheParams.itimAddr.nonEmpty && cacheParams.dataCode.canDetect).option(Valid(UInt(width = paddrBits))) //？
}

/*ICache：封装整个ICache作为top；
 *LazyModule解释：Lazy cache is a simple in-memory caching service. It has a developer friendly generics based API, and provides a thread safe cache implementation that guarantees to only execute your cachable delegates once (it’s lazy!). Under the hood it leverages ObjectCache and Lazy to provide performance and reliability in heavy load scenarios；
 *module：选择Lazy Cache；
 *masterNode/slaveNode：是针对Tilelink互联方式的masters/slave节点；
 *size：ICache的大小=nSets*nWays*blockBytes；
 *device：标记ICache说对应的设备号，如L1/L2/... I$；sifive是RISCV团队创建的公司名；
*/
class ICache(val icacheParams: ICacheParams, val hartId: Int)(implicit p: Parameters) extends LazyModule {
  lazy val module = new ICacheModule(this)
  val masterNode = TLClientNode(Seq(TLClientPortParameters(Seq(TLClientParameters(
    sourceId = IdRange(0, 1 + icacheParams.prefetch.toInt), // 0=refill, 1=hint, 0-1 + icacheParams.prefetch.toInt
    name = s"Core ${hartId} ICache")))))

  val size = icacheParams.nSets * icacheParams.nWays * icacheParams.blockBytes
  val device = new SimpleDevice("itim", Seq("sifive,itim0"))

  private val wordBytes = icacheParams.fetchBytes
//itimAddr包含address，resources，regionType，executable，supportsPutFull，supportsPutPartial，supportsGet，fifoId
  val slaveNode =
    TLManagerNode(icacheParams.itimAddr.toSeq.map { itimAddr => TLManagerPortParameters(
      Seq(TLManagerParameters(
        address         = Seq(AddressSet(itimAddr, size-1)),
        resources       = device.reg("mem"),
        regionType      = RegionType.IDEMPOTENT,
        executable      = true,
        supportsPutFull = TransferSizes(1, wordBytes),
        supportsPutPartial = TransferSizes(1, wordBytes),
        supportsGet     = TransferSizes(1, wordBytes),
        fifoId          = Some(0))), // requests handled in FIFO order
      beatBytes = wordBytes,
      minLatency = 1)})
}

/*ICacheResp：定义一个响应函数；
 *data：I$ HIT后输出的对应指令；
 *replay：I$重载；
 *ae（address extension)
*/
class ICacheResp(outer: ICache) extends Bundle {
  val data = UInt(width = outer.icacheParams.fetchBytes*8)
  val replay = Bool()
  val ae = Bool()

  override def cloneType = new ICacheResp(outer).asInstanceOf[this.type]
}

class ICachePerfEvents extends Bundle {
  val acquire = Bool()
}

/*ICacheBundle:定义I$在frontend中s1/s2阶段的使用参数；
 *hartid：指明具体的core；
 *req：请求信号；
 *s1_paddr：s1阶段经过TLB查找得到指令所在的物理地址；
 *s2_vaddr：s2阶段将当前虚拟地址返回至NPC Generation处产生npc；
 *s1_kill/s2_kill：s1/s2阶段作废；
 *s2_prefetch：是否支持预期指令；
 *resp：响应信号，取出相应指令后有效；
 *invalidate：I$失效信息，具体失效条件见下文；
*/
class ICacheBundle(val outer: ICache) extends CoreBundle()(outer.p) {
  val hartid = UInt(INPUT, hartIdLen)
  val req = Decoupled(new ICacheReq).flip
  val s1_paddr = UInt(INPUT, paddrBits) // delayed one cycle w.r.t. req
  val s2_vaddr = UInt(INPUT, vaddrBits) // delayed two cycles w.r.t. req
  val s1_kill = Bool(INPUT) // delayed one cycle w.r.t. req
  val s2_kill = Bool(INPUT) // delayed two cycles; prevents I$ miss emission
  val s2_prefetch = Bool(INPUT) // should I$ prefetch next line on a miss?

  val resp = Valid(new ICacheResp(outer))
  val invalidate = Bool(INPUT)

  val errors = new ICacheErrors
  val perf = new ICachePerfEvents().asOutput

  val clock_enabled = Bool(INPUT)
  val keep_clock_enabled = Bool(OUTPUT)
}

/*ICacheModule:I$具体工作情况
*/
class ICacheModule(outer: ICache) extends LazyModuleImp(outer)
    with HasL1ICacheParameters {
  //cacheParams可以覆盖icacheParams不带def的参数；
  override val cacheParams = outer.icacheParams // Use the local parameters

  val io = IO(new ICacheBundle(outer))
  
  //对Tile-link cached配置输入输出模式；
  //配置多个cache的主从模式，注意在TL下，TT（track tip）具有W/R权限；其他node有数据传输的功能；
  val (tl_out, edge_out) = outer.masterNode.out(0)  
  // Option.unzip does not exist :-(
  val (tl_in, edge_in) = outer.slaveNode.in.headOption.unzip
  
  val tECC = cacheParams.tagCode
  val dECC = cacheParams.dataCode

  //requir的作用类似assert；
  //两者的区别在于：	assert意味着你的程序已经达到了不一致的状态，这可能是目前的方法/函数的一个问题；
  //require意味着方法的调用者有问题，应该修复它的调用；
  require(isPow2(nSets) && isPow2(nWays))
  require(!usingVM || pgIdxBits >= untagBits)

  //scratchpadOn：I$是否使用Scratchpad memory；
  //scratchpadMax：Scratchpad memory的位数限制；
  //lineInScratchpad=（scratchpadOn && line <= scratchpadMax）,其中后半部分对数据进行判断，当scratchpadMax>=line时，为真；
  //scratchpadBase：配置itimAddr地址；
  val scratchpadOn = RegInit(false.B)
  val scratchpadMax = tl_in.map(tl => Reg(UInt(width = log2Ceil(nSets * (nWays - 1)))))
  def lineInScratchpad(line: UInt) = scratchpadMax.map(scratchpadOn && line <= _).getOrElse(false.B)
  val scratchpadBase = outer.icacheParams.itimAddr.map { dummy =>
    p(LookupByHartId)(_.icache.flatMap(_.itimAddr.map(_.U)), io.hartid)  //根据hartid寻找flatMap；
  }
  //addrMaybeInScratchpad：初判当前地址时候处于Scratchpad memory（provides	lowlatency	high-throughput	temporary	storage）内；
  //addrInScratchpad：终判当前地址时候处于Scratchpad memory内；
  //scratchpadWay/scratchpadLine：判断pc地址属于第几组第几列；
  //因为I$在整个Frontend均使用到，故有s0/s1/s2/s3有效状态；
  def addrMaybeInScratchpad(addr: UInt) = scratchpadBase.map(base => addr >= base && addr < base + outer.size).getOrElse(false.B)
  def addrInScratchpad(addr: UInt) = addrMaybeInScratchpad(addr) && lineInScratchpad(addr(untagBits+log2Ceil(nWays)-1, blockOffBits))
  def scratchpadWay(addr: UInt) = addr.extract(untagBits+log2Ceil(nWays)-1, untagBits)
  def scratchpadWayValid(way: UInt) = way < nWays - 1
  def scratchpadLine(addr: UInt) = addr(untagBits+log2Ceil(nWays)-1, blockOffBits)
  val s0_slaveValid = tl_in.map(_.a.fire()).getOrElse(false.B)
  val s1_slaveValid = RegNext(s0_slaveValid, false.B)
  val s2_slaveValid = RegNext(s1_slaveValid, false.B)
  val s3_slaveValid = RegNext(false.B)
  val s1_valid = Reg(init=Bool(false))
  
  //s1_tag_hit：判断pc在tag中是否已匹配；
  //s1_hit：pc对应I$的指令被取出的状态；
  //dontTouch(s1_hit)：这个指令是针对后端布局布线优化时忽略该线（涉及到时序）；
  val s1_tag_hit = Wire(Vec(nWays, Bool()))
  val s1_hit = s1_tag_hit.reduce(_||_) || Mux(s1_slaveValid, true.B, addrMaybeInScratchpad(io.s1_paddr))
  dontTouch(s1_hit)
  val s2_valid = RegNext(s1_valid && !io.s1_kill, Bool(false))
  val s2_hit = RegNext(s1_hit)

  val invalidated = Reg(Bool())
  val refill_valid = RegInit(false.B)
  val send_hint = RegInit(false.B)
 
  //refill_fire：判断I$中是否还有数据；*
  //s2_miss：s2阶段pc访问I$时是否miss？
  //s1_can_request_refill：当s2_miss/refill_valid有效时，s1阶段的数据无效，需要重新填充；
  //refill_addr/refill_tag/refill_idx：将s1阶段的物理地址/tag/idx重装载；
  //refill_one_beat：当前cache输出数据为空且边路cache仍存在数据时，置位；
  val refill_fire = tl_out.a.fire() && !send_hint
  val hint_outstanding = RegInit(false.B)
  val s2_miss = s2_valid && !s2_hit && !io.s2_kill
  val s1_can_request_refill = !(s2_miss || refill_valid)
  val s2_request_refill = s2_miss && RegNext(s1_can_request_refill)
  val refill_addr = RegEnable(io.s1_paddr, s1_valid && s1_can_request_refill)
  val refill_tag = refill_addr(tagBits+untagBits-1,untagBits) //（查找tag位置）；
  val refill_idx = refill_addr(untagBits-1,blockOffBits)
  val refill_one_beat = tl_out.d.fire() && edge_out.hasData(tl_out.d.bits)  

  io.req.ready := !(refill_one_beat || s0_slaveValid || s3_slaveValid)
  val s0_valid = io.req.fire()
  val s0_vaddr = io.req.bits.addr                            //即npc；
  s1_valid := s0_valid
  
  //edge_out.count(tl_out.d)对应赋值给(_, _, d_done, refill_cnt)；
  //refill_done：重填充完毕标志；
  val (_, _, d_done, refill_cnt) = edge_out.count(tl_out.d)  
  val refill_done = refill_one_beat && d_done  
  tl_out.d.ready := !s3_slaveValid
  require (edge_out.manager.minLatency > 0)

  //repl_way：pc数据送往scratchpad memeory还是旁路（直接到s2 reg）；
  //v0是16b线性反馈reg；
  val repl_way = if (isDM) UInt(0) else {
    // pick a way that is not used by the scratchpad
    val v0 = LFSR16(refill_fire)(log2Up(nWays)-1,0)  
    var v = v0  //？对于4路组关联内的取指令过程；
    for (i <- log2Ceil(nWays) - 1 to 0 by -1) {
      val mask = nWays - (BigInt(1) << (i + 1))
      v = v | (lineInScratchpad(Cat(v0 | mask.U, refill_idx)) << i)
    }
    assert(!lineInScratchpad(Cat(v, refill_idx)))
    v
  }
  
  //tag信息；
  val (tag_array, omSRAM) = DescribedSRAM(
    name = "tag_array",
    desc = "ICache Tag Array",
    size = nSets,
    data = Vec(nWays, UInt(width = tECC.width(1 + tagBits)))
  )
  //tag_rdata：vpc中tag位与I$中tag对比；括号内参数含义：（pc addr，有效条件）；
  //accruedRefillError：记录refill出错次数；
  val tag_rdata = tag_array.read(s0_vaddr(untagBits-1,blockOffBits), !refill_done && s0_valid)
  val accruedRefillError = Reg(Bool())
  //下面涉及到重填充后，对refill_tag检错，并配置I$中tag_array信息；
  when (refill_done) {
    // For AccessAckData, denied => corrupt
    val enc_tag = tECC.encode(Cat(tl_out.d.bits.corrupt, refill_tag))
    tag_array.write(refill_idx, Vec.fill(nWays)(enc_tag), Seq.tabulate(nWays)(repl_way === _))
    ccover(tl_out.d.bits.corrupt, "D_CORRUPT", "I$ D-channel corrupt")  //显示字符串；
  }
  
  //vb_array：数据存储的中间变量；
  val vb_array = Reg(init=Bits(0, nSets*nWays))
  when (refill_one_beat) {
    // clear bit（repl_way, refill_idx） when refill starts so hit-under-miss doesn't fetch bad data
    vb_array := vb_array.bitSet(Cat(repl_way, refill_idx), refill_done && !invalidated)
  }
  val invalidate = Wire(init = io.invalidate)
  when (invalidate) {
    vb_array := Bits(0)
    invalidated := Bool(true)
  }
  
  val s1_tag_disparity = Wire(Vec(nWays, Bool()))
  val s1_tl_error = Wire(Vec(nWays, Bool()))
  val wordBits = outer.icacheParams.fetchBytes*8
  val s1_dout = Wire(Vec(nWays, UInt(width = dECC.width(wordBits))))
  
  //定义各阶段的slaveAddr；
  val s0_slaveAddr = tl_in.map(_.a.bits.address).getOrElse(0.U)
  val s1s3_slaveAddr = Reg(UInt(width = log2Ceil(outer.size)))
  val s1s3_slaveData = Reg(UInt(width = wordBits))
  
  //pc中的tag与I$中4路tag匹配过程：
  //S1.对每一路逐个比较；
  //S2.若出现当前$为从设备无效状态，且vb_array中第s1_idx+i个元素为1，同时出现情况一：tag译码出错，则标记s1_tag_disparity；
  //S2.出现情况二：tag HIT，tilelink-cache未出错，则标记s1_tag_hit；
  //S2.出现情况三：tag HIT，且tilelink-cache出错，则标记s1_tl_error；
  //S3.当前pc处于scratchpadHit状态，则标记s1_tag_hit；
  for (i <- 0 until nWays) {
    val s1_idx = io.s1_paddr(untagBits-1,blockOffBits)
    val s1_tag = io.s1_paddr(tagBits+untagBits-1,untagBits)
    val scratchpadHit = scratchpadWayValid(i) &&
      Mux(s1_slaveValid,
        lineInScratchpad(scratchpadLine(s1s3_slaveAddr)) && scratchpadWay(s1s3_slaveAddr) === i,
        addrInScratchpad(io.s1_paddr) && scratchpadWay(io.s1_paddr) === i)
    val s1_vb = vb_array(Cat(UInt(i), s1_idx)) && !s1_slaveValid
    val enc_tag = tECC.decode(tag_rdata(i))
    val (tl_error, tag) = Split(enc_tag.uncorrected, tagBits)
    val tagMatch = s1_vb && tag === s1_tag
    s1_tag_disparity(i) := s1_vb && enc_tag.error
    s1_tl_error(i) := tagMatch && tl_error.asBool
    s1_tag_hit(i) := tagMatch || scratchpadHit
  }
  assert(!(s1_valid || s1_slaveValid) || PopCount(s1_tag_hit zip s1_tag_disparity map { case (h, d) => h && !d }) <= 1)

  require(tl_out.d.bits.data.getWidth % wordBits == 0)
  //data_arrays：I$ HIT后输出的指令数据；
  val data_arrays = Seq.tabulate(tl_out.d.bits.data.getWidth / wordBits) {
    i =>
      DescribedSRAM(
        name = s"data_arrays_${i}",
        desc = "ICache Data Array",
        size = nSets * refillCycles,
        data = Vec(nWays, UInt(width = dECC.width(wordBits)))
      )
  }
  //S4.将HIT的data_arrays打包赋值存在SRAM中的data_array；
  //推测：RISC-V采用TL-$结构，故I$有多个，但是master $只有一个，并且master $要保存HIT对应的data_array；
  //wordMatch：检测pc说对应的是哪一路数据；
  //row：计算共有多少行；
  //s0_ren(s0 read enable)：当s0阶段有效且已找到pc多对应的指令有效；
  //wen(write enable)
  //mem_idx:标记对应memory；
  for (((data_array, omSRAM), i) <- data_arrays zipWithIndex) {
    def wordMatch(addr: UInt) = addr.extract(log2Ceil(tl_out.d.bits.data.getWidth/8)-1, log2Ceil(wordBits/8)) === i
    def row(addr: UInt) = addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
    val s0_ren = (s0_valid && wordMatch(s0_vaddr)) || (s0_slaveValid && wordMatch(s0_slaveAddr))
    val wen = (refill_one_beat && !invalidated) || (s3_slaveValid && wordMatch(s1s3_slaveAddr))
    val mem_idx = Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
                  Mux(s3_slaveValid, row(s1s3_slaveAddr),
                  Mux(s0_slaveValid, row(s0_slaveAddr),
                  row(s0_vaddr))))
    //当写使能有效时，将对应数据写在I$的data_array中；              
    when (wen) {
      val data = Mux(s3_slaveValid, s1s3_slaveData, tl_out.d.bits.data(wordBits*(i+1)-1, wordBits*i))
      val way = Mux(s3_slaveValid, scratchpadWay(s1s3_slaveAddr), repl_way)
      data_array.write(mem_idx, Vec.fill(nWays)(dECC.encode(data)), (0 until nWays).map(way === _))
    }
    //dout就是输出的指令；
    val dout = data_array.read(mem_idx, !wen && s0_ren)
    when (wordMatch(Mux(s1_slaveValid, s1s3_slaveAddr, io.s1_paddr))) {
      s1_dout := dout
    }
  }
  //定义各阶段的I$状态；
  //s2_tag_hit:vpn找到对应的ppn；
  //s2_hit_way：HIT对应的路；
  //s2_scratchpad_word_addr：HIT对应位置输出对应地址；
  //s2_way_mux：输出数据的选择；
  val s1_clk_en = s1_valid || s1_slaveValid
  val s2_tag_hit = RegEnable(s1_tag_hit, s1_clk_en)
  val s2_hit_way = OHToUInt(s2_tag_hit)
  val s2_scratchpad_word_addr = Cat(s2_hit_way, Mux(s2_slaveValid, s1s3_slaveAddr, io.s2_vaddr)(untagBits-1, log2Ceil(wordBits/8)), UInt(0, log2Ceil(wordBits/8)))
  val s2_dout = RegEnable(s1_dout, s1_clk_en)
  val s2_way_mux = Mux1H(s2_tag_hit, s2_dout)
  //对tag和data进行检错与纠错；
  val s2_tag_disparity = RegEnable(s1_tag_disparity, s1_clk_en).asUInt.orR
  val s2_tl_error = RegEnable(s1_tl_error.asUInt.orR, s1_clk_en)
  val s2_data_decoded = dECC.decode(s2_way_mux)
  val s2_disparity = s2_tag_disparity || s2_data_decoded.error
  val s2_full_word_write = Wire(init = false.B)
  //当使用scratchpad memory且pc所指指令正好在该memory内，则产生s1/s2_scratchpad_hit；
  val s1_scratchpad_hit = Mux(s1_slaveValid, lineInScratchpad(scratchpadLine(s1s3_slaveAddr)), addrInScratchpad(io.s1_paddr))
  val s2_scratchpad_hit = RegEnable(s1_scratchpad_hit, s1_clk_en)
  val s2_report_uncorrectable_error = s2_scratchpad_hit && s2_data_decoded.uncorrectable && (s2_valid || (s2_slaveValid && !s2_full_word_write))
  val s2_error_addr = scratchpadBase.map(base => Mux(s2_scratchpad_hit, base + s2_scratchpad_word_addr, 0.U)).getOrElse(0.U)

  // output signals
  //依据说配置的latency确定各级I$的MISS情况，latency越大，输出数据所需的cycle越大；
  outer.icacheParams.latency match {
    case 1 =>
      require(tECC.isInstanceOf[IdentityCode])
      require(dECC.isInstanceOf[IdentityCode])
      require(outer.icacheParams.itimAddr.isEmpty)
      io.resp.bits.data := Mux1H(s1_tag_hit, s1_dout)
      io.resp.bits.ae := s1_tl_error.asUInt.orR
      io.resp.valid := s1_valid && s1_hit

    case 2 =>
      //当latency=2时，表明在s2阶段内，访问I$已经存在出错的情况；
      //输出的data为译码错误的数据；
      when (s2_valid && s2_disparity) { invalidate := true }

      io.resp.bits.data := s2_data_decoded.uncorrected
      io.resp.bits.ae := s2_tl_error
      io.resp.bits.replay := s2_disparity
      io.resp.valid := s2_valid && s2_hit
      //correctable/uncorrectable：判断该错误是不是可被纠正；
      io.errors.correctable.foreach { c =>
        c.valid := (s2_valid || s2_slaveValid) && s2_disparity && !s2_report_uncorrectable_error
        c.bits := s2_error_addr
      }
      io.errors.uncorrectable.foreach { u =>
        u.valid := s2_report_uncorrectable_error
        u.bits := s2_error_addr
      }
      //配置TL-$：对s1/s2/s3初始化；
      tl_in.map { tl =>
        val respValid = RegInit(false.B)
        tl.a.ready := !(tl_out.d.valid || s1_slaveValid || s2_slaveValid || s3_slaveValid || respValid || !io.clock_enabled)
        val s1_a = RegEnable(tl.a.bits, s0_slaveValid)
        //Latency=2时，I$在经历s2阶段的前需要把s1的数据全部读入；
        s2_full_word_write := edge_in.get.hasData(s1_a) && s1_a.mask.andR
        when (s0_slaveValid) {
        //当s0_slaveValid有效时，需要通过scratchpad memory，配置s1s3_slave的地址与数据；
          val a = tl.a.bits
          s1s3_slaveAddr := tl.a.bits.address
          s1s3_slaveData := tl.a.bits.data
          when (edge_in.get.hasData(a)) {
            val enable = scratchpadWayValid(scratchpadWay(a.address))
            when (!lineInScratchpad(scratchpadLine(a.address))) {
              scratchpadMax.get := scratchpadLine(a.address)
              invalidate := true
            }
            scratchpadOn := enable
            //不适用scratchpad memory时，需要通过分配地址的来找到pc对应的指令数据；
            val itim_allocated = !scratchpadOn && enable
            val itim_deallocated = scratchpadOn && !enable
            val itim_increase = scratchpadOn && enable && scratchpadLine(a.address) > scratchpadMax.get
            val refilling = refill_valid && refill_cnt > 0
            ccover(itim_allocated, "ITIM_ALLOCATE", "ITIM allocated")
            ccover(itim_allocated && refilling, "ITIM_ALLOCATE_WHILE_REFILL", "ITIM allocated while I$ refill")
            ccover(itim_deallocated, "ITIM_DEALLOCATE", "ITIM deallocated")
            ccover(itim_deallocated && refilling, "ITIM_DEALLOCATE_WHILE_REFILL", "ITIM deallocated while I$ refill")
            ccover(itim_increase, "ITIM_SIZE_INCREASE", "ITIM size increased")
            ccover(itim_increase && refilling, "ITIM_SIZE_INCREASE_WHILE_REFILL", "ITIM size increased while I$ refill")
          }
        }

        assert(!s2_valid || RegNext(RegNext(s0_vaddr)) === io.s2_vaddr)
        when (!(tl.a.valid || s1_slaveValid || s2_slaveValid || respValid)
              && s2_valid && s2_data_decoded.error && !s2_tag_disparity) {
          // handle correctable errors on CPU accesses to the scratchpad.
          // if there is an in-flight slave-port access to the scratchpad,
          // report the a miss but don't correct the error (as there is
          // a structural hazard on s1s3_slaveData/s1s3_slaveAddress).
          s3_slaveValid := true
          s1s3_slaveData := s2_data_decoded.corrected
          s1s3_slaveAddr := s2_scratchpad_word_addr | s1s3_slaveAddr(log2Ceil(wordBits/8)-1, 0)
        }
        
        respValid := s2_slaveValid || (respValid && !tl.d.ready)
        val respError = RegEnable(s2_scratchpad_hit && s2_data_decoded.uncorrectable && !s2_full_word_write, s2_slaveValid)
        when (s2_slaveValid) {
        //当s2阶段出多，则使能s3_slaveValid来借用s3时间，并在s3更新s1s3_slaveData；  
          when (edge_in.get.hasData(s1_a) || s2_data_decoded.error) { s3_slaveValid := true }
          def byteEn(i: Int) = !(edge_in.get.hasData(s1_a) && s1_a.mask(i))
          s1s3_slaveData := (0 until wordBits/8).map(i => Mux(byteEn(i), s2_data_decoded.corrected, s1s3_slaveData)(8*(i+1)-1, 8*i)).asUInt
        }
        //根据实际情况，输出I$执行结果；
        tl.d.valid := respValid
        tl.d.bits := Mux(edge_in.get.hasData(s1_a),
          edge_in.get.AccessAck(s1_a),  //？
          edge_in.get.AccessAck(s1_a, UInt(0), denied = Bool(false), corrupt = respError))
        tl.d.bits.data := s1s3_slaveData

        // Tie off unused channels
        tl.b.valid := false
        tl.c.ready := true
        tl.e.ready := true

        ccover(s0_valid && s1_slaveValid, "CONCURRENT_ITIM_ACCESS_1", "ITIM accessed, then I$ accessed next cycle")
        ccover(s0_valid && s2_slaveValid, "CONCURRENT_ITIM_ACCESS_2", "ITIM accessed, then I$ accessed two cycles later")
        ccover(tl.d.valid && !tl.d.ready, "ITIM_D_STALL", "ITIM response blocked by D-channel")
        ccover(tl_out.d.valid && !tl_out.d.ready, "ITIM_BLOCK_D", "D-channel blocked by ITIM access")
      }
  }
  //当refill时，a level TL-$输出有效；
  tl_out.a.valid := s2_request_refill
  tl_out.a.bits := edge_out.Get(
                    fromSource = UInt(0),
                    toAddress = (refill_addr >> blockOffBits) << blockOffBits,
                    lgSize = lgCacheBlockBytes)._2
  //需要指令预取时，需要根据refill_addr来找到预期指令所在位置并赋值给crosses_page，next_block；
  if (cacheParams.prefetch) {
    val (crosses_page, next_block) = Split(refill_addr(pgIdxBits-1, blockOffBits) +& 1, pgIdxBits-blockOffBits)
    //pc说对应的指令取完后，send_hint产生正脉冲，并输出数据；
    when (tl_out.a.fire()) {
      send_hint := !hint_outstanding && io.s2_prefetch && !crosses_page
      when (send_hint) {
        send_hint := false
        hint_outstanding := true
      }
    }
    when (refill_done) {
      send_hint := false
    }
    when (tl_out.d.fire() && !refill_one_beat) {
      hint_outstanding := false
    }

    when (send_hint) {
      tl_out.a.valid := true
      tl_out.a.bits := edge_out.Hint(
                        fromSource = UInt(1),
                        toAddress = Cat(refill_addr >> pgIdxBits, next_block) << blockOffBits,
                        lgSize = lgCacheBlockBytes,
                        param = TLHints.PREFETCH_READ)._2
    }

    ccover(send_hint && !tl_out.a.ready, "PREFETCH_A_STALL", "I$ prefetch blocked by A-channel")
    ccover(refill_valid && (tl_out.d.fire() && !refill_one_beat), "PREFETCH_D_BEFORE_MISS_D", "I$ prefetch resolves before miss")
    ccover(!refill_valid && (tl_out.d.fire() && !refill_one_beat), "PREFETCH_D_AFTER_MISS_D", "I$ prefetch resolves after miss")
    ccover(tl_out.a.fire() && hint_outstanding, "PREFETCH_D_AFTER_MISS_A", "I$ prefetch resolves after second miss")
  }
  //a级I$输出数据，b级I$处于准备状态；
  tl_out.b.ready := Bool(true)
  tl_out.c.valid := Bool(false)
  tl_out.e.valid := Bool(false)
  assert(!(tl_out.a.valid && addrMaybeInScratchpad(tl_out.a.bits.address)))

  when (!refill_valid) { invalidated := false.B }
  when (refill_fire) { refill_valid := true.B }
  when (refill_done) { refill_valid := false.B}
  //统计refill的次数，作为I$的perfomance event报告；
  io.perf.acquire := refill_fire
  //时钟使能条件：判别ITIM各有效情况和I$各有效情况；
  io.keep_clock_enabled :=
    tl_in.map(tl => tl.a.valid || tl.d.valid || s1_slaveValid || s2_slaveValid || s3_slaveValid).getOrElse(false.B) || // ITIM
    s1_valid || s2_valid || refill_valid || send_hint || hint_outstanding // I$

  ccover(!send_hint && (tl_out.a.valid && !tl_out.a.ready), "MISS_A_STALL", "I$ miss blocked by A-channel")
  ccover(invalidate && refill_valid, "FLUSH_DURING_MISS", "I$ flushed during miss")

  def ccover(cond: Bool, label: String, desc: String)(implicit sourceInfo: SourceInfo) =
    cover(cond, s"ICACHE_$label", "MemorySystem;;" + desc)

  val mem_active_valid = Seq(CoverBoolean(s2_valid, Seq("mem_active")))
  //根据s2阶段译码后correctable/uncorrectable清楚，输出data_error；
  val data_error = Seq( 
    CoverBoolean(!s2_data_decoded.correctable && !s2_data_decoded.uncorrectable, Seq("no_data_error")),
    CoverBoolean(s2_data_decoded.correctable, Seq("data_correctable_error")),
    CoverBoolean(s2_data_decoded.uncorrectable, Seq("data_uncorrectable_error")))
  //两种请求源："from_CPU"/"from_TL";
  val request_source = Seq(
    CoverBoolean(!s2_slaveValid, Seq("from_CPU")),
    CoverBoolean(s2_slaveValid, Seq("from_TL"))
  )
  val tag_error = Seq(
    CoverBoolean(!s2_tag_disparity, Seq("no_tag_error")),
    CoverBoolean(s2_tag_disparity, Seq("tag_error"))
  )
  val mem_mode = Seq(
    CoverBoolean(s2_scratchpad_hit, Seq("ITIM_mode")),
    CoverBoolean(!s2_scratchpad_hit, Seq("cache_mode"))
  )
  
  val error_cross_covers = new CrossProperty(
    Seq(mem_active_valid, data_error, tag_error, request_source, mem_mode),
    Seq(
      // tag error cannot occur in ITIM mode
      Seq("tag_error", "ITIM_mode"),
      // Can only respond to TL in ITIM mode
      Seq("from_TL", "cache_mode")
    ),
    "MemorySystem;;Memory Bit Flip Cross Covers")

  cover(error_cross_covers)
}
