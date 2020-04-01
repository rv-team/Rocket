// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket

import Chisel._
import Chisel.ImplicitConversions._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile.LookupByHartId
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import chisel3.{DontCare, WireInit}
import chisel3.internal.sourceinfo.SourceInfo
import chisel3.experimental._
import TLMessages._

// TODO: delete this trait once deduplication is smart enough to avoid globally inlining matching circuits
/*以下这个trait的作用是用于测试，可以直接忽略；*/
trait InlineInstance { self: chisel3.experimental.BaseModule =>  //self指代chisel3.experimental.BaseModule；
  chisel3.experimental.annotate(
    new chisel3.experimental.ChiselAnnotation {
      def toFirrtl: firrtl.annotations.Annotation = firrtl.passes.InlineAnnotation(self.toNamed) } )
}
/*DCacheErrors处理tag和data中的译码出错情况；*/
class DCacheErrors(implicit p: Parameters) extends L1HellaCacheBundle()(p)
    with CanHaveErrors {
  val correctable = (cacheParams.tagCode.canCorrect || cacheParams.dataCode.canCorrect).option(Valid(UInt(width = paddrBits)))//针对实地址中的每一位码位做检查；
  val uncorrectable = (cacheParams.tagCode.canDetect || cacheParams.dataCode.canDetect).option(Valid(UInt(width = paddrBits)))
  val bus = Valid(UInt(width = paddrBits))  //bus传输地址信息；
}
/*DCacheDataReq：外部请求信息；*/
class DCacheDataReq(implicit p: Parameters) extends L1HellaCacheBundle()(p) {
  val addr = Bits(width = untagBits)                      //外部请求的地址不包含tag信息；
  val write = Bool()                                      //当write被置位时，D$相应位置被写入wdata；
  val wdata = UInt(width = encBits * rowBytes / eccBytes)
  val wordMask = UInt(width = rowBytes / wordBytes)       //实现以字为单位提取所需信息；
  val eccMask = UInt(width = wordBytes / eccBytes)        //针对ECC，提取一个单位的信息；
  val way_en = Bits(width = nWays)                        //使能某一路；
}
/*DCacheDataArray：将req和resp捆绑成输入输出*/
class DCacheDataArray(implicit p: Parameters) extends L1HellaCacheModule()(p) {
  val io = new Bundle {
    val req = Valid(new DCacheDataReq).flip               //将其定义为输入模式；
    val resp = Vec(nWays, UInt(width = req.bits.wdata.getWidth)).asOutput
  }                                                       //响应时，resp输出包含了总路数以及数据；

  require(rowBytes % wordBytes == 0)
  val eccMask = if (eccBits == wordBits) Seq(true.B) else io.req.bits.eccMask.asBools
  val wMask = if (nWays == 1) eccMask else (0 until nWays).flatMap(i => eccMask.map(_ && io.req.bits.way_en(i)))  //依据wMask来找出eccMsk对应所需的那一路；
  val wWords = io.req.bits.wdata.grouped(encBits * (wordBits / eccBits))  //将wdata以字为单元变为wWords；
  val addr = io.req.bits.addr >> rowOffBits                               //对请求信号中的地址通过移位的方式得到D$中对应地址；
  val data_arrays = Seq.tabulate(rowBytes / wordBytes) {                  //以表格的形式将D$的信息表达出来；
    i =>
      DescribedSRAM(
        name = s"data_arrays_${i}",
        desc = "DCache Data Array",
        size = nSets * cacheBlockBytes / rowBytes,
        data = Vec(nWays * (wordBits / eccBits), UInt(width = encBits))
      )
  }
//对data_arrays中的array和omSRAM进行循环访问，在每次loop时，会赋值valid/data、查询是否为写操作/读操作；

//zipWithIndex/zip为一个循环计数器；
  val rdata = for (((array, omSRAM), i) <- data_arrays zipWithIndex) yield {    
    val valid = io.req.valid && (Bool(data_arrays.size == 1) || io.req.bits.wordMask(i))
    when (valid && io.req.bits.write) {
      val wData = wWords(i).grouped(encBits)                              //当为写操作时，索引addr上写入wData；
      array.write(addr, Vec((0 until nWays).flatMap(i => wData)), wMask)
    }
    val data = array.read(addr, valid && !io.req.bits.write)              //当为读操作时，依据addr读出data，并输出；
    data.grouped(wordBits / eccBits).map(_.asUInt).toSeq
  }
  (io.resp zip rdata.transpose).foreach { case (resp, data) => resp := data.asUInt }
}
/*DCacheMetadataReq，对data寄存器的请求访问；*/
class DCacheMetadataReq(implicit p: Parameters) extends L1HellaCacheBundle()(p) {
  val write = Bool()
  val addr = UInt(width = vaddrBitsExtended)
  val idx = UInt(width = idxBits)
  val way_en = UInt(width = nWays)
  val data = UInt(width = cacheParams.tagCode.width(new L1Metadata().getWidth))  //这里的data指的是tag信息；
}

class DCache(hartid: Int, val crossing: ClockCrossingType)(implicit p: Parameters) extends HellaCache(hartid)(p) {
  override lazy val module = new DCacheModule(this)
}

@chiselName
//tl_a/tl_b/tl_c/tl_d为tilelink的四个通道；
class DCacheModule(outer: DCache) extends HellaCacheModule(outer) {
  val tECC = cacheParams.tagCode                                    
  val dECC = cacheParams.dataCode
  require(isPow2(eccBytes) && eccBytes <= wordBytes)
  require(eccBytes == 1 || !dECC.isInstanceOf[IdentityCode])
  val usingRMW = eccBytes > 1 || usingAtomicsInCache                //查询是否使用了Atomics RMW 指令；
  val mmioOffset = outer.firstMMIO                                  //设置memory-mapped I/O的偏移量；

  val clock_en_reg = Reg(Bool())                                    //D$的时钟同步于cpu；
  io.cpu.clock_enabled := clock_en_reg

  val gated_clock =                                                
    if (!cacheParams.clockGate) clock
    else ClockGate(clock, clock_en_reg, "dcache_clock_gate")
  withClock (gated_clock) 
  { // entering gated-clock domain          //D$内部也划分流水线进行操作；

  // tags相关信息：
  val replacer = cacheParams.replacement                            //选择替换策略：random/PLRU/LRU;
  val metaArb = Module(new Arbiter(new DCacheMetadataReq, 8) with InlineInstance) //metaArb用于仲裁出某路的选择，类似tag，但是metaArb支持非阻塞时对D$多次访问；

  val (tag_array, omSRAM) = DescribedSRAM(
    name = "tag_array",
    desc = "DCache Tag Array",
    size = nSets,
    data = Vec(nWays, metaArb.io.out.bits.data)
  )

  // data
  val data = Module(new DCacheDataArray)
  val dataArb = Module(new Arbiter(new DCacheDataReq, 4) with InlineInstance)   //用于data间的仲裁，猜测用于解决cache一致性问题；
  dataArb.io.in.tail.foreach(_.bits.wdata := dataArb.io.in.head.bits.wdata)     //将io内数据写入数据wdata；
  data.io.req <> dataArb.io.out
  dataArb.io.out.ready := true                                                  //数据准备好后，dataArb/metaArb可用；
  metaArb.io.out.ready := clock_en_reg

  val tl_out_a = Wire(tl_out.a)                                                 //配置用于D$的tl类型；tl_out_a配置crossing；
  tl_out.a <> {
    val a_queue_depth = outer.crossing match {
      case RationalCrossing(_) => // TODO make this depend on the actual ratio?
        if (cacheParams.separateUncachedResp) (maxUncachedInFlight + 1) / 2
        else 2 min maxUncachedInFlight-1
      case SynchronousCrossing(BufferParams.none) => 1 // Need some buffering to guarantee livelock freedom
      case SynchronousCrossing(_) => 0 // Adequate buffering within the crossing
      case _: AsynchronousCrossing => 0 // Adequate buffering within the crossing
    }
    Queue(tl_out_a, a_queue_depth, flow = true)
  }

  val (tl_out_c, release_queue_empty) =
    if (cacheParams.acquireBeforeRelease) {
      val q = Module(new Queue(tl_out.c.bits.cloneType, cacheDataBeats, flow = true)) //当出现acquireBeforeRelease时，q存放tl_out中数据；
      tl_out.c <> q.io.deq
      (q.io.enq, q.io.count === 0)
    } else {
      (tl_out.c, true.B)
    }

  val s1_valid = Reg(next=io.cpu.req.fire(), init=Bool(false))  //刷新信号fire有效时，是s1有效；
  val s1_probe = Reg(next=tl_out.b.fire(), init=Bool(false))    //s1_probe是由prober发出；作用是检测rep_dirty ::rep_clean :: retry :: miss；
  val probe_bits = RegEnable(tl_out.b.bits, tl_out.b.fire())    
  val s1_nack = Wire(init=Bool(false))
  val s1_valid_masked = s1_valid && !io.cpu.s1_kill
  val s1_valid_not_nacked = s1_valid && !s1_nack
  val s1_req = Reg(io.cpu.req.bits)
  val s0_clk_en = metaArb.io.out.valid && !metaArb.io.out.bits.write//当metaArb没有写操作时，s1_req.addr中的地址按照metaarb与req地址相关联；
  when (s0_clk_en) {
    s1_req := io.cpu.req.bits
    s1_req.addr := Cat(metaArb.io.out.bits.addr >> blockOffBits, io.cpu.req.bits.addr(blockOffBits-1,0))  
    when (!metaArb.io.in(7).ready) { s1_req.phys := true }
  }
  val s1_read = isRead(s1_req.cmd)                              //根据req中的cmd来区分是写操作还是读操作；针对主存，还存在SFENCE操作；
  val s1_write = isWrite(s1_req.cmd)
  val s1_readwrite = s1_read || s1_write
  val s1_sfence = s1_req.cmd === M_SFENCE
  val s1_flush_line = s1_req.cmd === M_FLUSH_ALL && s1_req.size(0)//指定刷新行；
  val s1_flush_valid = Reg(Bool())
  val s1_waw_hazard = Wire(Bool())                                //标记WAW冒险；
  //将D$会出现的状态列举出来；
  val s_ready :: s_voluntary_writeback :: s_probe_rep_dirty :: s_probe_rep_clean :: s_probe_retry :: s_probe_rep_miss :: s_voluntary_write_meta :: s_probe_write_meta :: Nil = Enum(UInt(), 8)
  val supports_flush = outer.flushOnFenceI || coreParams.haveCFlush//出现Fence指令或者$自身需要刷新时，supports_flush有效；
  val flushed = Reg(init=Bool(true))                               //刷新的两种状态：已刷新和正在刷新；
  val flushing = Reg(init=Bool(false))
  val cached_grant_wait = Reg(init=Bool(false))                    //当出现多个D$访问请求时，若此时的D$正在工作，则cached_grant_wait置位；
  val release_ack_wait = Reg(init=Bool(false))
  val release_ack_addr = Reg(UInt(paddrBits.W))
  val can_acquire_before_release = !release_ack_wait && release_queue_empty//当释放队列为空且release处于响应状态时，can_acquire_before_release有效；TL-C中的一种操作，释放是由master发出，将共享数据发送给slave；
  val release_state = Reg(init=s_ready)
  val any_pstore_valid = Wire(Bool())                               //针对写操作的数据存储相关指令；
  val inWriteback = release_state.isOneOf(s_voluntary_writeback, s_probe_rep_dirty)//检测D$是直写还是写回策略；
  val releaseWay = Wire(UInt())
  io.cpu.req.ready := (release_state === s_ready) && !cached_grant_wait && !s1_nack

  // I/O MSHRs
  val uncachedInFlight = RegInit(Vec.fill(maxUncachedInFlight)(false.B))//MSHRs中正在处理的uncached操作，可并行操作；
  val uncachedReqs = Reg(Vec(maxUncachedInFlight, new HellaCacheReq))   
  val uncachedResp = WireInit(new HellaCacheReq, DontCare)

  // hit initiation path,for read cmd;
  //s0阶段在获取cmd/addr/关心位/way_en/ready状态并赋值给dataArb；
  val s0_read = isRead(io.cpu.req.bits.cmd)
  dataArb.io.in(3).valid := io.cpu.req.valid && likelyNeedsRead(io.cpu.req.bits)
  dataArb.io.in(3).bits := dataArb.io.in(1).bits
  dataArb.io.in(3).bits.write := false
  dataArb.io.in(3).bits.addr := io.cpu.req.bits.addr
  dataArb.io.in(3).bits.wordMask := UIntToOH(io.cpu.req.bits.addr.extract(rowOffBits-1,offsetlsb))//只关心addr中(rowOffBits-1,offsetlsb)位；
  dataArb.io.in(3).bits.way_en := ~UInt(0, nWays)
  when (!dataArb.io.in(3).ready && s0_read) { io.cpu.req.ready := false }
  //当dataArb所需状态准备好后，s1标记好s1_did_read并查询相应状态赋值给metaArb；
  val s1_did_read = RegEnable(dataArb.io.in(3).ready && (io.cpu.req.valid && needsRead(io.cpu.req.bits)), s0_clk_en)
  metaArb.io.in(7).valid := io.cpu.req.valid
  metaArb.io.in(7).bits.write := false
  metaArb.io.in(7).bits.idx := io.cpu.req.bits.addr(idxMSB, idxLSB)
  metaArb.io.in(7).bits.addr := io.cpu.req.bits.addr
  metaArb.io.in(7).bits.way_en := metaArb.io.in(4).bits.way_en
  metaArb.io.in(7).bits.data := metaArb.io.in(4).bits.data
  when (!metaArb.io.in(7).ready) { io.cpu.req.ready := false }

  // address translation
  val tlb = Module(new TLB(false, log2Ceil(coreDataBytes), TLBConfig(nTLBEntries)))
  val s1_cmd_uses_tlb = s1_readwrite || s1_flush_line                   //当出现读写操作或者刷新某行操作时，标记使用s1_cmd_uses_tlb来查询地址转换；
  io.ptw <> tlb.io.ptw                                                  //tlb与ptw相连；当s2阶段失效后，位于s2处理的tlb也失效；
  tlb.io.kill := io.cpu.s2_kill
  tlb.io.req.valid := s1_valid && !io.cpu.s1_kill && s1_cmd_uses_tlb    //普通情况下，tlb有效情况；
  tlb.io.req.bits.passthrough := s1_req.phys                            //当传入tlb中的地址已经是物理地址时，直接旁路tlb输出该地址；
  tlb.io.req.bits.vaddr := s1_req.addr                                
  tlb.io.req.bits.size := s1_req.size
  tlb.io.req.bits.cmd := s1_req.cmd
  when (!tlb.io.req.ready && !tlb.io.ptw.resp.valid && !io.cpu.req.bits.phys) { io.cpu.req.ready := false }
  when (s1_valid && s1_cmd_uses_tlb && tlb.io.resp.miss) { s1_nack := true }//当出现tlb缺失时，s1未响应信号s1_nack为真；
  //SFENCE Instruction:     funct7            rs2             rs1         funct3       rd         opcode
  //字段                31 ---------- 25 24 -------- 20 19 --------- 15 14----- 12 11-------- 7 6--------0
  tlb.io.sfence.valid := s1_valid && !io.cpu.s1_kill && s1_sfence         //处于sfence期间，tlb有效的情况；
//rs1=x0,rs2=x0,为所有地址空间对页表的任何级别的所有读和写进行排序；
//rs1=x0,rs2=/=x0,对页表的任何级别执行所有读和写操作，但只对整数寄存器rs2标识的地址空间执行；
//rs1=/=x0,rs2=x0，fence只对对应于rs1中的虚拟地址的叶页表条目进行读写；
//rs1=/=x0,rs2=/=x0,对于整数寄存器rs2标识的地址空间，fence只对对应于rs1中的虚拟地址的叶页表条目进行读写。
  tlb.io.sfence.bits.rs1 := s1_req.size(0)                                
  tlb.io.sfence.bits.rs2 := s1_req.size(1)                                
  tlb.io.sfence.bits.asid := io.cpu.s1_data.data                          //写操作对应的数据；
  tlb.io.sfence.bits.addr := s1_req.addr

  val s1_paddr = tlb.io.resp.paddr                    
  val s1_victim_way = Wire(init = replacer.way)                           //被替换的行就为victim_way；
  val (s1_hit_way, s1_hit_state, s1_meta, s1_victim_meta) =               //对4个变量的赋值过程就是{}中内容，D$中还涉及到了SPM；
    if (usingDataScratchpad) {
      val baseAddr = p(LookupByHartId)(_.dcache.flatMap(_.scratch.map(_.U)), io.hartid)
      val inScratchpad = s1_paddr >= baseAddr && s1_paddr < baseAddr + nSets * cacheBlockBytes
      val hitState = Mux(inScratchpad, ClientMetadata.maximum, ClientMetadata.onReset)
      val dummyMeta = L1Metadata(UInt(0), ClientMetadata.onReset)
      (inScratchpad, hitState, Seq(tECC.encode(dummyMeta.asUInt)), dummyMeta)
    } else {                                                                        //当ppc地址位于D$中时；
      val metaReq = metaArb.io.out                                                  
      val metaIdx = metaReq.bits.idx
      when (metaReq.valid && metaReq.bits.write) {                                  //执行写操作时，按照所指定的nWays将data写入tag；
        val wmask = if (nWays == 1) Seq(true.B) else metaReq.bits.way_en.asBools
        tag_array.write(metaIdx, Vec.fill(nWays)(metaReq.bits.data), wmask)
      }
      val s1_meta = tag_array.read(metaIdx, metaReq.valid && !metaReq.bits.write)   //执行读操作时，s1_meta读取tag信息；
      val s1_meta_uncorrected = s1_meta.map(tECC.decode(_).uncorrected.asTypeOf(new L1Metadata))
      val s1_tag = s1_paddr >> tagLSB                                               //s1_tag指的是物理地址中的tag字段，而meta就是指存在D$中的tag+有效信息；
      val s1_meta_hit_way = s1_meta_uncorrected.map(r => r.coh.isValid() && r.tag === s1_tag).asUInt//tag对比过程；
      val s1_meta_hit_state = ClientMetadata.onReset.fromBits(                      //meta中每一路的状态统计；
        s1_meta_uncorrected.map(r => Mux(r.tag === s1_tag && !s1_flush_valid, r.coh.asUInt, UInt(0)))
        .reduce (_|_))
      (s1_meta_hit_way, s1_meta_hit_state, s1_meta, s1_meta_uncorrected(s1_victim_way))
    }
  val s1_data_way = Wire(init = if (nWays == 1) 1.U else Mux(inWriteback, releaseWay, s1_hit_way))//数据所在的位置，若处于写回状态下，该数据在releaseway中；正常情况下在hit对应位置下；初试情况是默认选择在第一路；
  val tl_d_data_encoded = Wire(encodeData(tl_out.d.bits.data, false.B).cloneType)   
  val s1_all_data_ways = Vec(data.io.resp ++ (!cacheParams.separateUncachedResp).option(tl_d_data_encoded))
  val s1_mask_xwr = new StoreGen(s1_req.size, s1_req.addr, UInt(0), wordBytes).mask               //s1_mask_xwr和s1_data.mask都是利用mask提取addr中所需数据；
  val s1_mask = Mux(s1_req.cmd === M_PWR, io.cpu.s1_data.mask, s1_mask_xwr)                       
  // for partial writes, s1_data.mask must be a subset of s1_mask_xwr
  assert(!(s1_valid_masked && s1_req.cmd === M_PWR) || (s1_mask_xwr | ~io.cpu.s1_data.mask).andR)

  val s2_valid = Reg(next=s1_valid_masked && !s1_sfence, init=Bool(false))                        
  val s2_valid_no_xcpt = s2_valid && !io.cpu.s2_xcpt.asUInt.orR                                   
  val s2_probe = Reg(next=s1_probe, init=Bool(false))                                             //每个阶段都需要一个探针来实时监测此阶段的运行情况；
  val releaseInFlight = s1_probe || s2_probe || release_state =/= s_ready                         
  val s2_valid_masked = s2_valid_no_xcpt && Reg(next = !s1_nack)
  val s2_valid_not_killed = s2_valid_masked && !io.cpu.s2_kill                                    //s2阶段有效信号s2_valid_not_killed；
  val s2_req = Reg(io.cpu.req.bits)
  val s2_cmd_flush_all = s2_req.cmd === M_FLUSH_ALL && !s2_req.size(0)
  val s2_cmd_flush_line = s2_req.cmd === M_FLUSH_ALL && s2_req.size(0)
  val s2_tlb_resp = Reg(tlb.io.resp.cloneType)                                                    //此处会检查传来的地址是属于cacheable还是uncacheable‘
  val s2_uncached = !s2_tlb_resp.cacheable || s2_req.no_alloc && !s2_tlb_resp.must_alloc
  val s2_uncached_resp_addr = Reg(s2_req.addr.cloneType) // should be DCE'd in synthesis
  when (s1_valid_not_nacked || s1_flush_valid) {                                                  //当s1阶段被刷新或者是未响应时，需要在借助一个时钟周期即s2阶段继续访问D$；
    s2_req := s1_req
    s2_req.addr := s1_paddr
    s2_tlb_resp := tlb.io.resp
  }
  val s2_vaddr = Cat(RegEnable(s1_req.addr, s1_valid_not_nacked || s1_flush_valid) >> pgIdxBits, s2_req.addr(pgIdxBits-1, 0))//因为s1未执行完访问，故在s2阶段所需的地址依旧是vaddr；
  val s2_read = isRead(s2_req.cmd)                                                                //重复s1阶段做的工作，cmd识别；
  val s2_write = isWrite(s2_req.cmd)
  val s2_readwrite = s2_read || s2_write
  val s2_flush_valid_pre_tag_ecc = RegNext(s1_flush_valid)
  val s1_meta_decoded = s1_meta.map(tECC.decode(_))
  val s1_meta_clk_en = s1_valid_not_nacked || s1_flush_valid || s1_probe
  val s2_meta_correctable_errors = s1_meta_decoded.map(m => RegEnable(m.correctable, s1_meta_clk_en)).asUInt      //经过ECC机制，检查s1产生的meta数据是否correctable/uncorrectable；
  val s2_meta_uncorrectable_errors = s1_meta_decoded.map(m => RegEnable(m.uncorrectable, s1_meta_clk_en)).asUInt
  val s2_meta_error_uncorrectable = s2_meta_uncorrectable_errors.orR
  val s2_meta_corrected = s1_meta_decoded.map(m => RegEnable(m.corrected, s1_meta_clk_en).asTypeOf(new L1Metadata))
  val s2_meta_error = (s2_meta_uncorrectable_errors | s2_meta_correctable_errors).orR                             //统计meta error；
  val s2_flush_valid = s2_flush_valid_pre_tag_ecc && !s2_meta_error
  val s2_data = {
    val wordsPerRow = rowBits / wordBits                                                                          //意思就是data的大小，以字为单位；
    val en = s1_valid || inWriteback || io.cpu.replay_next
    //若查询到当前访问处于写回状态，则规定word_en=1；若为写操作，word_en=0；若为读操作，word_en=读取数据/wordsPerRow； ->word_en确定选中该路；
    val word_en = Mux(inWriteback, Fill(wordsPerRow, 1.U), Mux(!s1_did_read, 0.U, UIntToOH(s1_req.addr.extract((rowBits/8).log2-1, wordBytes.log2), wordsPerRow)))
    val s1_way_words = s1_all_data_ways.map(_.grouped(dECC.width(eccBits) * (wordBits / eccBits)))
    if (cacheParams.pipelineWayMux) {
      val s1_word_en = Mux(io.cpu.replay_next, 0.U, word_en)                                                     //出现replay指令时，s1_word_en=0；      (for (i <- 0 until wordsPerRow) yield {
        val s2_way_en = RegEnable(Mux(s1_word_en(i), s1_data_way, 0.U), en)                                      //根据s1_way来确定s2_way_en；
        val s2_way_words = (0 until nWays).map(j => RegEnable(s1_way_words(j)(i), en))                           //s2_way_words为对应data；
        (0 until nWays).map(j => Mux(s2_way_en(j), s2_way_words(j), 0.U)).reduce(_|_)
      }).asUInt
    } else {
      val s1_word_en = Mux(!io.cpu.replay_next, word_en, UIntToOH(uncachedResp.addr.extract(log2Up(rowBits/8)-1, log2Up(wordBytes)), wordsPerRow))
      (for (i <- 0 until wordsPerRow) yield {
        RegEnable(Mux1H(Mux(s1_word_en(i), s1_data_way, 0.U), s1_way_words.map(_(i))), en)
      }).asUInt
    }
  }
  val s2_probe_way = RegEnable(s1_hit_way, s1_probe)                                                              //RegEnable(next,(init,)enable);
  val s2_probe_state = RegEnable(s1_hit_state, s1_probe)                                                          //出现probe是从master获取shared block中的信息，含dirty data；
  val s2_hit_way = RegEnable(s1_hit_way, s1_valid_not_nacked)
  val s2_hit_state = RegEnable(s1_hit_state, s1_valid_not_nacked || s1_flush_valid)
  val s2_waw_hazard = RegEnable(s1_waw_hazard, s1_valid_not_nacked)
  val s2_store_merge = Wire(Bool())                                                                               //store_merge写合并？
  val s2_hit_valid = s2_hit_state.isValid()
  val (s2_hit, s2_grow_param, s2_new_hit_state) = s2_hit_state.onAccess(s2_req.cmd)                               //获取s2_req.cmd说对应的state以及tl tree grow方式，即s2_new_hit_state；
  val s2_data_decoded = decodeData(s2_data)
  val s2_word_idx = s2_req.addr.extract(log2Up(rowBits/8)-1, log2Up(wordBytes))                                   
  val s2_data_error = s2_data_decoded.map(_.error).orR
  val s2_data_error_uncorrectable = s2_data_decoded.map(_.uncorrectable).orR
  val s2_data_corrected = (s2_data_decoded.map(_.corrected): Seq[UInt]).asUInt
  val s2_data_uncorrected = (s2_data_decoded.map(_.uncorrected): Seq[UInt]).asUInt
  val s2_valid_hit_maybe_flush_pre_data_ecc_and_waw = s2_valid_masked && !s2_meta_error && s2_hit                 //？
  val s2_valid_hit_pre_data_ecc_and_waw = s2_valid_hit_maybe_flush_pre_data_ecc_and_waw && s2_readwrite
  val s2_valid_flush_line = s2_valid_hit_maybe_flush_pre_data_ecc_and_waw && s2_cmd_flush_line && can_acquire_before_release
  val s2_valid_hit_pre_data_ecc = s2_valid_hit_pre_data_ecc_and_waw && (!s2_waw_hazard || s2_store_merge)
  val s2_valid_data_error = s2_valid_hit_pre_data_ecc_and_waw && s2_data_error && can_acquire_before_release      //统计s2发生的data error/cached_miss等等基本情况；
  val s2_valid_hit = s2_valid_hit_pre_data_ecc && !s2_data_error
  val s2_valid_miss = s2_valid_masked && s2_readwrite && !s2_meta_error && !s2_hit && can_acquire_before_release
  val s2_valid_cached_miss = s2_valid_miss && !s2_uncached && !uncachedInFlight.asUInt.orR                        
  dontTouch(s2_valid_cached_miss)
  val s2_want_victimize = Bool(!usingDataScratchpad) && (s2_valid_cached_miss || s2_valid_flush_line || s2_valid_data_error || s2_flush_valid)
  val s2_cannot_victimize = !s2_flush_valid && io.cpu.s2_kill
  val s2_victimize = s2_want_victimize && !s2_cannot_victimize                                                    //s2_victimize是D$中是否替换某行的标记位；
  val s2_valid_uncached_pending = s2_valid_miss && s2_uncached && !uncachedInFlight.asUInt.andR                   //针对仍未处理的uncached map，标记成s2_valid_uncached_pending；
  val s2_victim_way = Mux(s2_hit_valid, s2_hit_way, UIntToOH(RegEnable(s1_victim_way, s1_valid_not_nacked || s1_flush_valid)))//当s2_hit_valid有效时，s2_victim_way/s2_victim_tag/s2_victim_state=s2_hit_way/...；反之，则为s2_victim_way//s2_victim_tag/s2_victim_state=s1_victim_way/...；
  val s2_victim_tag = Mux(s2_valid_data_error || s2_valid_flush_line, s2_req.addr(paddrBits-1, tagLSB), RegEnable(s1_victim_meta.tag, s1_valid_not_nacked || s1_flush_valid))
  val s2_victim_state = Mux(s2_hit_valid, s2_hit_state, RegEnable(s1_victim_meta.coh, s1_valid_not_nacked || s1_flush_valid))

  val (s2_prb_ack_data, s2_report_param, probeNewCoh)= s2_probe_state.onProbe(probe_bits.param)                   //当s2发送probe操作后的响应为(s2_prb_ack_data, s2_report_param, probeNewCoh)；
  val (s2_victim_dirty, s2_shrink_param, voluntaryNewCoh) = s2_victim_state.onCacheControl(M_FLUSH)               //当出现M_FLUSH：write back dirty data and cede R/W permissions时，标记(s2_victim_dirty, s2_shrink_param, voluntaryNewCoh)；
  dontTouch(s2_victim_dirty)
  val s2_update_meta = s2_hit_state =/= s2_new_hit_state                                                          //属于grow transition时，新状态与当前状态不一致时，需要更新状态；
  val s2_dont_nack_uncached = s2_valid_uncached_pending && tl_out_a.ready                                         //当有uncached处于等待状态且tl处于准备状态时，可响应uncached；
  val s2_dont_nack_flush = supports_flush && !s2_meta_error && (s2_cmd_flush_all && flushed && !flushing || s2_cmd_flush_line && !s2_hit)
  io.cpu.s2_nack := s2_valid_no_xcpt && !s2_dont_nack_uncached && !s2_dont_nack_flush && !s2_valid_hit
  when (io.cpu.s2_nack || (s2_valid_hit_pre_data_ecc_and_waw && s2_update_meta)) { s1_nack := true }

  // tag updates on ECC errors
  //出现meta_error_uncorrectable时，更新tag并将tl状态置位nothing（不可读不可写）；
  val s2_first_meta_corrected = PriorityMux(s2_meta_correctable_errors, s2_meta_corrected)
  metaArb.io.in(1).valid := s2_meta_error && (s2_valid_masked || s2_flush_valid_pre_tag_ecc || s2_probe)
  metaArb.io.in(1).bits.write := true
  metaArb.io.in(1).bits.way_en := s2_meta_uncorrectable_errors | Mux(s2_meta_error_uncorrectable, 0.U, PriorityEncoderOH(s2_meta_correctable_errors))
  metaArb.io.in(1).bits.idx := Mux(s2_probe, probeIdx(probe_bits), s2_vaddr(idxMSB, idxLSB))
  metaArb.io.in(1).bits.addr := Cat(io.cpu.req.bits.addr >> untagBits, metaArb.io.in(1).bits.idx << blockOffBits)
  metaArb.io.in(1).bits.data := tECC.encode {
    val new_meta = Wire(init = s2_first_meta_corrected)
    when (s2_meta_error_uncorrectable) { new_meta.coh := ClientMetadata.onReset }                               
    new_meta.asUInt
  }

  // tag updates on hit for write；
  metaArb.io.in(2).valid := s2_valid_hit_pre_data_ecc_and_waw && s2_update_meta
  metaArb.io.in(2).bits.write := !s2_data_error && !io.cpu.s2_kill
  metaArb.io.in(2).bits.way_en := s2_victim_way
  metaArb.io.in(2).bits.idx := s2_vaddr(idxMSB, idxLSB)
  metaArb.io.in(2).bits.addr := Cat(io.cpu.req.bits.addr >> untagBits, s2_vaddr(idxMSB, 0))
  metaArb.io.in(2).bits.data := tECC.encode(L1Metadata(s2_req.addr >> tagLSB, s2_new_hit_state).asUInt)

  // load reservations and TL error reporting
  val s2_lr = Bool(usingAtomics && !usingDataScratchpad) && s2_req.cmd === M_XLR
  val s2_sc = Bool(usingAtomics && !usingDataScratchpad) && s2_req.cmd === M_XSC
  val lrscCount = Reg(init=UInt(0))                                               //lrsc：Low Retention STT-MRAM Cache
  val lrscValid = lrscCount > lrscBackoff
  val lrscBackingOff = lrscCount > 0 && !lrscValid
  val lrscAddr = Reg(UInt())
  val lrscAddrMatch = lrscAddr === (s2_req.addr >> blockOffBits)                  //查询s2_req.addr是否在lrsc中；
  val s2_sc_fail = s2_sc && !(lrscValid && lrscAddrMatch)
  when ((s2_valid_hit && s2_lr && !cached_grant_wait || s2_valid_cached_miss) && !io.cpu.s2_kill) {
    lrscCount := Mux(s2_hit, lrscCycles - 1, 0.U)
    lrscAddr := s2_req.addr >> blockOffBits
  }
  when (lrscCount > 0) { lrscCount := lrscCount - 1 }                           
  when (s2_valid_not_killed && lrscValid) { lrscCount := lrscBackoff }
  when (s1_probe) { lrscCount := 0 }

  // don't perform data correction if it might clobber a recent store
  val s2_correct = s2_data_error && !any_pstore_valid && !RegNext(any_pstore_valid) && Bool(usingDataScratchpad)//s2_correct标记需要经过ECC的data；
  // pending store buffer
  val s2_valid_correct = s2_valid_hit_pre_data_ecc_and_waw && s2_correct && !io.cpu.s2_kill
  def s2_store_valid_pre_kill = s2_valid_hit && s2_write && !s2_sc_fail
  def s2_store_valid = s2_store_valid_pre_kill && !io.cpu.s2_kill                                 //针对写操作，先识别cmd，在提取addr/wdata/hit_way/mask；
  val pstore1_cmd = RegEnable(s1_req.cmd, s1_valid_not_nacked && s1_write)
  val pstore1_addr = RegEnable(s1_req.addr, s1_valid_not_nacked && s1_write)
  val pstore1_data = RegEnable(io.cpu.s1_data.data, s1_valid_not_nacked && s1_write)
  val pstore1_way = RegEnable(s1_hit_way, s1_valid_not_nacked && s1_write)
  val pstore1_mask = RegEnable(s1_mask, s1_valid_not_nacked && s1_write)
  val pstore1_storegen_data = Wire(init = pstore1_data)
  val pstore1_rmw = Bool(usingRMW) && RegEnable(needsRead(s1_req), s1_valid_not_nacked && s1_write)//使用AUTOMIC RWM指令下的存储；
  val pstore1_merge_likely = s2_valid && s2_write && s2_store_merge
  val pstore1_merge = s2_store_valid && s2_store_merge
  val pstore2_valid = Reg(Bool())                                                                 //是否使用第二个store buffer；
  val pstore_drain_opportunistic = !(io.cpu.req.valid && likelyNeedsRead(io.cpu.req.bits)) && !(s1_valid && s1_waw_hazard)//出现waw冒险或者读操作情况下，会导致D$ store buffer drain；
  val pstore_drain_on_miss = releaseInFlight || RegNext(io.cpu.s2_nack)                           //当出现release操作或者s2无响应的情况下，会导致D$ store buffer drain miss；
  val pstore1_held = Reg(Bool())
  val pstore1_valid_likely = s2_valid && s2_write || pstore1_held
  def pstore1_valid_not_rmw(s2_kill: Bool) = s2_valid_hit_pre_data_ecc && s2_write && !s2_kill || pstore1_held//不使用rmw情况下的pstore1；
  val pstore1_valid = s2_store_valid || pstore1_held
  any_pstore_valid := pstore1_held || pstore2_valid
  val pstore_drain_structural = pstore1_valid_likely && pstore2_valid && ((s1_valid && s1_write) || pstore1_rmw)//pstore_drain_structural：D$出现结构性冒险；
  assert(pstore1_rmw || pstore1_valid_not_rmw(io.cpu.s2_kill) === pstore1_valid)
  ccover(pstore_drain_structural, "STORE_STRUCTURAL_HAZARD", "D$ read-modify-write structural hazard")
  ccover(pstore1_valid && pstore_drain_on_miss, "STORE_DRAIN_ON_MISS", "D$ store buffer drain on miss")
  ccover(s1_valid_not_nacked && s1_waw_hazard, "WAW_HAZARD", "D$ write-after-write hazard")
  def should_pstore_drain(truly: Bool) = {                                                        //should_pstore_drain：是否将store buffer drain；
    val s2_kill = truly && io.cpu.s2_kill
    !pstore1_merge_likely &&
    (Bool(usingRMW) && pstore_drain_structural ||
      (((pstore1_valid_not_rmw(s2_kill) && !pstore1_rmw) || pstore2_valid) && (pstore_drain_opportunistic || pstore_drain_on_miss)))
  }
  val pstore_drain = should_pstore_drain(true)
  pstore1_held := (s2_store_valid && !s2_store_merge || pstore1_held) && pstore2_valid && !pstore_drain
  val advance_pstore1 = (pstore1_valid || s2_valid_correct) && (pstore2_valid === pstore_drain)   
  pstore2_valid := pstore2_valid && !pstore_drain || advance_pstore1
  val pstore2_addr = RegEnable(Mux(s2_correct, s2_vaddr, pstore1_addr), advance_pstore1)
  val pstore2_way = RegEnable(Mux(s2_correct, s2_hit_way, pstore1_way), advance_pstore1)
  val pstore2_storegen_data = {                                                                 //pstore2所存储的data，基本流程类似pstore1；
    for (i <- 0 until wordBytes)
      yield RegEnable(pstore1_storegen_data(8*(i+1)-1, 8*i), advance_pstore1 || pstore1_merge && pstore1_mask(i))
  }.asUInt
  val pstore2_storegen_mask = {
    val mask = Reg(UInt(width = wordBytes))
    when (advance_pstore1 || pstore1_merge) {
      val mergedMask = pstore1_mask | Mux(pstore1_merge, mask, 0.U)
      mask := ~Mux(s2_correct, 0.U, ~mergedMask)
    }
    mask
  }
  s2_store_merge := (if (eccBytes == 1) false.B else {                                        //D$ store merged；
    ccover(pstore1_merge, "STORE_MERGED", "D$ store merged")
    // only merge stores to ECC granules that are already stored-to, to avoid WAW hazards
    val wordMatch = (eccMask(pstore2_storegen_mask) | ~eccMask(pstore1_mask)).andR            //pstore1与pstore2存储内容匹配；
    val idxMatch = s2_vaddr(untagBits-1, log2Ceil(wordBytes)) === pstore2_addr(untagBits-1, log2Ceil(wordBytes))
    val tagMatch = (s2_hit_way & pstore2_way).orR
    pstore2_valid && wordMatch && idxMatch && tagMatch
  })
  dataArb.io.in(0).valid := should_pstore_drain(false)                                        //根据pstore1与pstore2来配置dataArb；
  dataArb.io.in(0).bits.write := pstore_drain
  dataArb.io.in(0).bits.addr := Mux(pstore2_valid, pstore2_addr, pstore1_addr)
  dataArb.io.in(0).bits.way_en := Mux(pstore2_valid, pstore2_way, pstore1_way)
  dataArb.io.in(0).bits.wdata := encodeData(Fill(rowWords, Mux(pstore2_valid, pstore2_storegen_data, pstore1_data)), false.B)
  dataArb.io.in(0).bits.wordMask := UIntToOH(Mux(pstore2_valid, pstore2_addr, pstore1_addr).extract(rowOffBits-1,offsetlsb))
  dataArb.io.in(0).bits.eccMask := eccMask(Mux(pstore2_valid, pstore2_storegen_mask, pstore1_mask))

  // store->load RAW hazard detection
  def s1Depends(addr: UInt, mask: UInt) =                                                    //s1Depends获取s1阶段执行写操作的地址和mask信息；
    addr(idxMSB, wordOffBits) === s1_req.addr(idxMSB, wordOffBits) &&
    Mux(s1_write, (eccByteMask(mask) & eccByteMask(s1_mask_xwr)).orR, (mask & s1_mask_xwr).orR)
  val s1_hazard =                                                                            //获取写操作在D$上的写位置，因为写操作耗时长，直接标记为s1_hazard；
    (pstore1_valid_likely && s1Depends(pstore1_addr, pstore1_mask)) ||
     (pstore2_valid && s1Depends(pstore2_addr, pstore2_storegen_mask))
  val s1_raw_hazard = s1_read && s1_hazard                                                   //同一块位置又出现读/写操作，标记为s1_raw_hazard/s1_waw_hazard；                                                  
  s1_waw_hazard := (if (eccBytes == 1) false.B else {
    ccover(s1_valid_not_nacked && s1_waw_hazard, "WAW_HAZARD", "D$ write-after-write hazard")
    s1_write && (s1_hazard || needsRead(s1_req) && !s1_did_read)
  })
  when (s1_valid && s1_raw_hazard) { s1_nack := true }

  // performance hints to processor
  io.cpu.s2_nack_cause_raw := RegNext(s1_raw_hazard) || !(!s2_waw_hazard || s2_store_merge)

  // Prepare a TileLink request message that initiates a transaction，涉及到了cache一致性问题，借助TL-C协议来对主从设备的shared data访问；
  //使用TL_A通道来完成PutFullData/PutPartialData/ArithmeticData/LogicalData/Get/Hint/AcquireBlock/AcquirePerm；
  val a_source = PriorityEncoder(~uncachedInFlight.asUInt << mmioOffset) // skip the MSHR
  val acquire_address = (s2_req.addr >> idxLSB) << idxLSB                //master对s2_req.addr对应的slave设备发送acquire；
  val access_address = s2_req.addr                                       //master对s2_req.addr对应的slave设备发送access；
  val a_size = s2_req.size
  val a_data = Fill(beatWords, pstore1_data)
  val get     = edge.Get(a_source, access_address, a_size)._2           //a_source从access_address获取a_size大小的数据；
  val put     = edge.Put(a_source, access_address, a_size, a_data)._2   //a_source把a_size大小的a_data数据传给access_address；
  val atomics = if (edge.manager.anySupportLogical) {
    MuxLookup(s2_req.cmd, Wire(new TLBundleA(edge.bundle)), Array(      //将数据传输给支持原子计算的managers（功能单元）；
      M_XA_SWAP -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.SWAP)._2,
      M_XA_XOR  -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.XOR) ._2,
      M_XA_OR   -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.OR)  ._2,
      M_XA_AND  -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.AND) ._2,
      M_XA_ADD  -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.ADD)._2,
      M_XA_MIN  -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MIN)._2,
      M_XA_MAX  -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MAX)._2,
      M_XA_MINU -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MINU)._2,
      M_XA_MAXU -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MAXU)._2))
  } else {
    // If no managers support atomics, assert fail if processor asks for them
    assert (!(tl_out_a.valid && s2_read && s2_write && s2_uncached))
    Wire(new TLBundleA(edge.bundle))
  }
  //tl传输有效情况的定义以及tl传输的目标地址/原地址/数据/put or get操作；
  tl_out_a.valid := !io.cpu.s2_kill && ((s2_valid_cached_miss && (Bool(cacheParams.acquireBeforeRelease) || !s2_victim_dirty)) || s2_valid_uncached_pending)
  tl_out_a.bits := Mux(!s2_uncached, acquire(s2_vaddr, s2_req.addr, s2_grow_param), Mux(!s2_write, get, Mux(!s2_read, put, atomics)))

  // Set pending bits for outstanding TileLink transaction
  val a_sel = UIntToOH(a_source, maxUncachedInFlight+mmioOffset) >> mmioOffset
  when (tl_out_a.fire()) {
    when (s2_uncached) {
      (a_sel.asBools zip (uncachedInFlight zip uncachedReqs)) foreach { case (s, (f, r)) =>
        when (s) {
          f := Bool(true)
          r := s2_req
        }
      }
    }.otherwise {
      cached_grant_wait := true
    }
  }

  // grant
  //使用TL_D通道来完成AccessAck/AccessAckData/HintAck/Grant/GrantData/ReleaseAck操作；
  val (d_first, d_last, d_done, d_address_inc) = edge.addr_inc(tl_out.d)    
  val (d_opc, grantIsUncached, grantIsUncachedData) = {                     //d_opc区分对于uncached是Grant/GrantData这样的TLMessages；
    val uncachedGrantOpcodesSansData = Seq(AccessAck, HintAck)              //AccessAck向原始请求代理提供一个无数据确认；HintAck用作提示操作的确认响应；
    val uncachedGrantOpcodesWithData = Seq(AccessAckData)                   //AccessAckData向原始请求代理提供一个带有数据的确认；
    val uncachedGrantOpcodes = uncachedGrantOpcodesWithData ++ uncachedGrantOpcodesSansData
    val whole_opc = tl_out.d.bits.opcode
    if (usingDataScratchpad) {
      assert(!tl_out.d.valid || whole_opc.isOneOf(uncachedGrantOpcodes))
      // the only valid TL-D messages are uncached, so we can do some pruning
      val opc = whole_opc(uncachedGrantOpcodes.map(_.getWidth).max - 1, 0)
      val data = DecodeLogic(opc, uncachedGrantOpcodesWithData, uncachedGrantOpcodesSansData)
      (opc, true.B, data)
    } else {
      (whole_opc, whole_opc.isOneOf(uncachedGrantOpcodes), whole_opc.isOneOf(uncachedGrantOpcodesWithData))
    }
  }
  tl_d_data_encoded := encodeData(tl_out.d.bits.data, tl_out.d.bits.corrupt && !grantIsUncached)      //对data进行译码；
  val grantIsCached = d_opc.isOneOf(Grant, GrantData)
  val grantIsVoluntary = d_opc === ReleaseAck // Clears a different pending bit
  val grantIsRefill = d_opc === GrantData     // Writes the data array
  val grantInProgress = Reg(init=Bool(false))
  val blockProbeAfterGrantCount = Reg(init=UInt(0))                                                   //在Grant信息之后，延迟处理Probe数；
  when (blockProbeAfterGrantCount > 0) { blockProbeAfterGrantCount := blockProbeAfterGrantCount - 1 }
  val canAcceptCachedGrant = !release_state.isOneOf(s_voluntary_writeback, s_voluntary_write_meta)    //当D$不处于writeback和write_meta操作时，可接受master对其的Grant操作；
  tl_out.d.ready := Mux(grantIsCached, (!d_first || tl_out.e.ready) && canAcceptCachedGrant, true.B)  //D通道可输出的准备信号；
  val uncachedRespIdxOH = UIntToOH(tl_out.d.bits.source, maxUncachedInFlight+mmioOffset) >> mmioOffset//查询这条被发送到ti.out的uncached map具体位置；
  uncachedResp := Mux1H(uncachedRespIdxOH, uncachedReqs)
  when (tl_out.d.fire()) {
    when (grantIsCached) {                  //当此条grant在$中仍在处理时，需要等待tl.out中d_last处理结束，方可对D通道重新fire填充；
      grantInProgress := true                                                                   
      assert(cached_grant_wait, "A GrantData was unexpected by the dcache.")
      when(d_last) {
        cached_grant_wait := false
        grantInProgress := false
        blockProbeAfterGrantCount := blockProbeAfterGrantCycles - 1
        replacer.miss//？
      }
    } .elsewhen (grantIsUncached) {        //当此条grant是针对Uncached，等待tl.out中d_last处理结束；
      (uncachedRespIdxOH.asBools zip uncachedInFlight) foreach { case (s, f) =>
        when (s && d_last) {
          assert(f, "An AccessAck was unexpected by the dcache.") // TODO must handle Ack coming back on same cycle!
          f := false
        }
      }
      when (grantIsUncachedData) {        //GrantData消息既是响应也是请求消息，它由从属代理用于向原始请求主代理提供确认以及数据块的副本；
        if (!cacheParams.separateUncachedResp) {
          if (!cacheParams.pipelineWayMux)
            s1_data_way := 1.U << nWays
          s2_req.cmd := M_XRD
          s2_req.size := uncachedResp.size
          s2_req.signed := uncachedResp.signed
          s2_req.tag := uncachedResp.tag
          s2_req.addr := {
            require(rowOffBits >= beatOffBits)
            val dontCareBits = s1_paddr >> rowOffBits << rowOffBits
            dontCareBits | uncachedResp.addr(beatOffBits-1, 0)
          }
          s2_uncached_resp_addr := uncachedResp.addr
        }
      }
    } .elsewhen (grantIsVoluntary) {
      assert(release_ack_wait, "A ReleaseAck was unexpected by the dcache.") // TODO should handle Ack coming back on same cycle!
      release_ack_wait := false
    }
  }

  // Finish TileLink transaction by issuing a GrantAck（位于E通道）；
  tl_out.e.valid := tl_out.d.valid && d_first && grantIsCached && canAcceptCachedGrant
  tl_out.e.bits := edge.GrantAck(tl_out.d.bits)
  assert(tl_out.e.fire() === (tl_out.d.fire() && d_first && grantIsCached))

  // data refill
  //注意这个准备有效的信号忽略了E通道backpressure，这有利于数据RAM有时可能被冗余写入；
  dataArb.io.in(1).valid := tl_out.d.valid && grantIsRefill && canAcceptCachedGrant
  when (grantIsRefill && !dataArb.io.in(1).ready) {
    tl_out.e.valid := false
    tl_out.d.ready := false                                                 //关闭D通道的ready信号，开始重填充dataArb；
  }
  if (!usingDataScratchpad) {                                               //正常填充D$中data信息，需要借助写操作/写地址/wdata/有效信息wordMask；
    dataArb.io.in(1).bits.write := true
    dataArb.io.in(1).bits.addr :=  (s2_vaddr >> idxLSB) << idxLSB | d_address_inc
    dataArb.io.in(1).bits.way_en := s2_victim_way
    dataArb.io.in(1).bits.wdata := tl_d_data_encoded
    dataArb.io.in(1).bits.wordMask := ~UInt(0, rowBytes / wordBytes)
    dataArb.io.in(1).bits.eccMask := ~UInt(0, wordBytes / eccBytes)
  } else {
    dataArb.io.in(1).bits := dataArb.io.in(0).bits                           //当使用SPM时，dataArb.io.in(1)=dataArb.io.in(0)；
  }

  // tag updates on refill
  //metaArb backpressure <- 只能由hit-under-miss下的tag ECC错误引起;
  //未能写入新标签会使该行无效，因此我们稍后会再次请求该行；
  metaArb.io.in(3).valid := grantIsCached && d_done && !tl_out.d.bits.denied  //对metaArb重填充时，需要写操作/写地址/refill数据；
  metaArb.io.in(3).bits.write := true
  metaArb.io.in(3).bits.way_en := s2_victim_way
  metaArb.io.in(3).bits.idx := s2_vaddr(idxMSB, idxLSB)
  metaArb.io.in(3).bits.addr := Cat(io.cpu.req.bits.addr >> untagBits, s2_vaddr(idxMSB, 0))
  metaArb.io.in(3).bits.data := tECC.encode(L1Metadata(s2_req.addr >> tagLSB, s2_hit_state.onGrant(s2_req.cmd, tl_out.d.bits.param)).asUInt)

  if (!cacheParams.separateUncachedResp) {
    // don't accept uncached grants if there's a structural hazard on s2_data...
    val blockUncachedGrant = Reg(Bool())
    blockUncachedGrant := dataArb.io.out.valid
    when (grantIsUncachedData && (blockUncachedGrant || s1_valid)) {
      tl_out.d.ready := false
      // ...but insert bubble to guarantee grant's eventual forward progress
      when (tl_out.d.valid) {
        io.cpu.req.ready := false
        dataArb.io.in(1).valid := true
        dataArb.io.in(1).bits.write := false
        blockUncachedGrant := !dataArb.io.in(1).ready
      }
    }
  }
  ccover(tl_out.d.valid && !tl_out.d.ready, "BLOCK_D", "D$ D-channel blocked")

  // Handle an incoming TileLink Probe message
  //延迟处理probe的几种情况：
  //1.当blockProbeAfterGrantCount还有效或是使用lrsc这类core_progress时；
  //2.出现release_ack悬挂状态或是release正在处理；
  val block_probe_for_core_progress = blockProbeAfterGrantCount > 0 || lrscValid    
  val block_probe_for_pending_release_ack = release_ack_wait && (tl_out.b.bits.address ^ release_ack_addr)(idxMSB, idxLSB) === 0
  val block_probe_for_ordering = releaseInFlight || block_probe_for_pending_release_ack || grantInProgress
  metaArb.io.in(6).valid := tl_out.b.valid && (!block_probe_for_core_progress || lrscBackingOff)  //对metaArb读数据时，需要读操作/地址；
  tl_out.b.ready := metaArb.io.in(6).ready && !(block_probe_for_core_progress || block_probe_for_ordering || s1_valid || s2_valid)
  metaArb.io.in(6).bits.write := false
  metaArb.io.in(6).bits.idx := probeIdx(tl_out.b.bits)
  metaArb.io.in(6).bits.addr := Cat(io.cpu.req.bits.addr >> paddrBits, tl_out.b.bits.address)
  metaArb.io.in(6).bits.way_en := metaArb.io.in(4).bits.way_en
  metaArb.io.in(6).bits.data := metaArb.io.in(4).bits.data                                        //读操作涉及到的data具体是什么？

  // release（C通道）；由master发出该信号，将shared block（分批传输，涉及到releaseDataBeat）写回slave设备中；
  val (c_first, c_last, releaseDone, c_count) = edge.count(tl_out_c)
  val releaseRejected = tl_out_c.valid && !tl_out_c.ready
  val s1_release_data_valid = Reg(next = dataArb.io.in(2).fire())                                 //D$的s1/s2阶段分别都可release操作；
  val s2_release_data_valid = Reg(next = s1_release_data_valid && !releaseRejected)
  val releaseDataBeat = Cat(UInt(0), c_count) + Mux(releaseRejected, UInt(0), s1_release_data_valid + Cat(UInt(0), s2_release_data_valid))
  val nackResponseMessage = edge.ProbeAck(b = probe_bits, reportPermissions = TLPermissions.NtoN) //无数据传输；
  val cleanReleaseMessage = edge.ProbeAck(b = probe_bits, reportPermissions = s2_report_param)
  val dirtyReleaseMessage = edge.ProbeAck(b = probe_bits, reportPermissions = s2_report_param, data = 0.U)//写回dirty bit；

  tl_out_c.valid := s2_release_data_valid                                                         //C通道先发送nackResponseMessage；
  tl_out_c.bits := nackResponseMessage                                                    
  val newCoh = Wire(init = probeNewCoh)
  releaseWay := s2_probe_way

  if (!usingDataScratchpad) {                                                                     //当处于s2_valid_cached_miss || s2_valid_flush_line || s2_valid_data_error || s2_flush_valid时，
    when (s2_victimize) {                                                                         //release某含脏块的block时，release_state为s_voluntary_writeback/s_voluntary_write_meta；
      assert(s2_valid_flush_line || s2_flush_valid || io.cpu.s2_nack)
      release_state := Mux(s2_victim_dirty, s_voluntary_writeback, s_voluntary_write_meta)
      probe_bits := addressToProbe(s2_vaddr, Cat(s2_victim_tag, s2_req.addr(tagLSB-1, idxLSB)) << idxLSB)
    }
    when (s2_probe) {                                                                             //当处于probe状态下，meta出错，则release_state=s_probe_retry，需要再次访问该设备直至其ready；
      val probeNack = Wire(init = true.B)
      when (s2_meta_error) {                                                        
        release_state := s_probe_retry
      }.elsewhen (s2_prb_ack_data) {                                                              //出现probe后，再出现ProbeAckData，则release_state=s_probe_rep_dirty；
        release_state := s_probe_rep_dirty                                                        //ProbeAckData消息是MASTER使用的响应消息，用于确认接收到探针并回写请求从代理所需的脏数据;
      }.elsewhen (s2_probe_state.isValid()) {                                                     //所需的release_state准备完毕后，1.可以开始release操作，即将共享块/permissions传输给slave设备；
        tl_out_c.valid := true                                                                    //传输完毕后，仍要更新release_state为s_probe_write_meta/s_probe_rep_clean；
        tl_out_c.bits := cleanReleaseMessage                                                      //2.release仍处于无效阶段时，更新release_state为s_ready/s_probe_rep_miss
        release_state := Mux(releaseDone, s_probe_write_meta, s_probe_rep_clean)
      }.otherwise {
        tl_out_c.valid := true
        probeNack := !releaseDone
        release_state := Mux(releaseDone, s_ready, s_probe_rep_miss)
      }
      when (probeNack) { s1_nack := true }
    }
    when (release_state === s_probe_retry) {                                                      //当release_state=s_probe_retry，需要再次访问该设备直至其ready；                                               
      metaArb.io.in(6).valid := true
      metaArb.io.in(6).bits.idx := probeIdx(probe_bits)
      metaArb.io.in(6).bits.addr := Cat(io.cpu.req.bits.addr >> paddrBits, probe_bits.address)
      when (metaArb.io.in(6).ready) {
        release_state := s_ready
        s1_probe := true
      }
    }
    when (release_state === s_probe_rep_miss) {                                                   //若release_state=s_probe_rep_miss，使能C通道，
      tl_out_c.valid := true                                                                      //等待release执行完毕后，release_state更新至s_ready，等待执行probe；
      when (releaseDone) { release_state := s_ready }
    }
    when (release_state === s_probe_rep_clean) {                                                  //若release_state=s_probe_rep_clean，使能C通道，发送清空release measage给C通道，
      tl_out_c.valid := true                                                                      //等待release执行完毕后，release_states_probe_write_meta，借助probe来清空；
      tl_out_c.bits := cleanReleaseMessage
      when (releaseDone) { release_state := s_probe_write_meta }
    }
    when (release_state === s_probe_rep_dirty) {                                                  //若release_state=s_probe_rep_dirty，通知C通道将要执行发送dirty block；
      tl_out_c.bits := dirtyReleaseMessage                                                        //等待release执行完毕后，release_states_probe_write_meta，借助probe来清空；
      when (releaseDone) { release_state := s_probe_write_meta }
    }
    when (release_state.isOneOf(s_voluntary_writeback, s_voluntary_write_meta)) {                 //若release_state=s_voluntary_writeback/s_voluntary_write_meta，真正开始执行release，
      tl_out_c.bits := edge.Release(fromSource = 0.U,                                             //配置release操作所需参数；
                                    toAddress = 0.U,
                                    lgSize = lgCacheBlockBytes,
                                    shrinkPermissions = s2_shrink_param,
                                    data = 0.U)._2
      newCoh := voluntaryNewCoh
      releaseWay := s2_victim_way
      when (releaseDone) { release_state := s_voluntary_write_meta }                              //等待release执行完毕后，更新release_state=s_voluntary_write_meta；
      when (tl_out_c.fire() && c_first) {                                                         //当release还未执行完毕时出现清空C通道情况：
        release_ack_wait := true                                                                  //1.release等待信号release_ack_wait置位；
        release_ack_addr := probe_bits.address                                                    //2.指明release继续执行时的地址release_ack_addr；
      }
    }
    tl_out_c.bits.source := probe_bits.source                                                     //若release_state= s_ready/s_probe_retry/s_probe_write_meta，
    tl_out_c.bits.address := probe_bits.address                                                   //配置probe操作所需参数；
    tl_out_c.bits.data := s2_data_corrected
    tl_out_c.bits.corrupt := inWriteback && s2_data_error_uncorrectable
  }

  dataArb.io.in(2).valid := inWriteback && releaseDataBeat < refillCycles                         //dataArb.io.in(2)为probe操作中发送方；
  dataArb.io.in(2).bits := dataArb.io.in(1).bits
  dataArb.io.in(2).bits.write := false
  dataArb.io.in(2).bits.addr := (probeIdx(probe_bits) << blockOffBits) | (releaseDataBeat(log2Up(refillCycles)-1,0) << rowOffBits)
  dataArb.io.in(2).bits.wordMask := ~UInt(0, rowBytes / wordBytes)
  dataArb.io.in(2).bits.way_en := ~UInt(0, nWays)

  metaArb.io.in(4).valid := release_state.isOneOf(s_voluntary_write_meta, s_probe_write_meta)     //metaArb.io.in(4)为probe操作中接受方；
  metaArb.io.in(4).bits.write := true
  metaArb.io.in(4).bits.way_en := releaseWay
  metaArb.io.in(4).bits.idx := probeIdx(probe_bits)
  metaArb.io.in(4).bits.addr := Cat(io.cpu.req.bits.addr >> untagBits, probe_bits.address(idxMSB, 0))
  metaArb.io.in(4).bits.data := tECC.encode(L1Metadata(tl_out_c.bits.address >> tagLSB, newCoh).asUInt)
  when (metaArb.io.in(4).fire()) { release_state := s_ready }

  // cached response
  io.cpu.resp.bits <> s2_req
  io.cpu.resp.bits.has_data := s2_read
  io.cpu.resp.bits.replay := false
  io.cpu.s2_uncached := s2_uncached && !s2_hit
  io.cpu.s2_paddr := s2_req.addr

  // report whether there are any outstanding accesses.  disregard any
  // slave-port accesses, since they don't affect local memory ordering.
  val s1_isSlavePortAccess = usingDataScratchpad && s1_req.phys
  val s2_isSlavePortAccess = usingDataScratchpad && s2_req.phys
  io.cpu.ordered := !(s1_valid && !s1_isSlavePortAccess || s2_valid && !s2_isSlavePortAccess || cached_grant_wait || uncachedInFlight.asUInt.orR)

  val s1_xcpt_valid = tlb.io.req.valid && !s1_nack
  io.cpu.s2_xcpt := Mux(RegNext(s1_xcpt_valid), s2_tlb_resp, 0.U.asTypeOf(s2_tlb_resp))

  if (usingDataScratchpad) {
    require(!usingVM) // therefore, req.phys means this is a slave-port access
    when (s2_isSlavePortAccess) {
      assert(!s2_valid || s2_hit_valid)
      io.cpu.s2_xcpt := 0.U.asTypeOf(io.cpu.s2_xcpt)
    }
    assert(!(s2_valid_masked && s2_req.cmd.isOneOf(M_XLR, M_XSC)))
  } else {
    ccover(tl_out.b.valid && !tl_out.b.ready, "BLOCK_B", "D$ B-channel blocked")
  }

  // uncached response
  val s1_uncached_data_word = {
    val word_idx = uncachedResp.addr.extract(log2Up(rowBits/8)-1, log2Up(wordBytes))                //提取uncached resp中data信息；
    val words = tl_out.d.bits.data.grouped(wordBits)
    words(word_idx)
  }
  val s2_uncached_data_word = RegEnable(s1_uncached_data_word, io.cpu.replay_next)       
  val doUncachedResp = Reg(next = io.cpu.replay_next)
  io.cpu.resp.valid := (s2_valid_hit_pre_data_ecc || doUncachedResp) && !s2_data_error
  io.cpu.replay_next := tl_out.d.fire() && grantIsUncachedData && !cacheParams.separateUncachedResp
  when (doUncachedResp) {                                                                           //当访问uncached，cpu响应时需要replay，具体为什么做未知？
    assert(!s2_valid_hit)
    io.cpu.resp.bits.replay := true
    io.cpu.resp.bits.addr := s2_uncached_resp_addr
  }

  io.cpu.uncached_resp.map { resp =>                                                                
    resp.valid := tl_out.d.valid && grantIsUncachedData
    resp.bits.tag := uncachedResp.tag
    resp.bits.size := uncachedResp.size
    resp.bits.signed := uncachedResp.signed
    resp.bits.data := new LoadGen(uncachedResp.size, uncachedResp.signed, uncachedResp.addr, s1_uncached_data_word, false.B, wordBytes).data
    when (grantIsUncachedData && !resp.ready) {
      tl_out.d.ready := false
    }
  }

  // load data subword mux/sign extension，针对在s2阶段的s2.req来执行的load指令，并将结果作为io.cpu.resp响应；
  val s2_data_word = (0 until rowBits by wordBits).map(i => s2_data_uncorrected(wordBits+i-1,i)).reduce(_|_)
  val s2_data_word_corrected = (0 until rowBits by wordBits).map(i => s2_data_corrected(wordBits+i-1,i)).reduce(_|_)
  val s2_data_word_possibly_uncached = Mux(cacheParams.pipelineWayMux && doUncachedResp, s2_uncached_data_word, 0.U) | s2_data_word
  val loadgen = new LoadGen(s2_req.size, s2_req.signed, s2_req.addr, s2_data_word_possibly_uncached, s2_sc, wordBytes)
  io.cpu.resp.bits.data := loadgen.data | s2_sc_fail
  io.cpu.resp.bits.data_word_bypass := loadgen.wordData
  io.cpu.resp.bits.data_raw := s2_data_word
  io.cpu.resp.bits.store_data := pstore1_data

  // AMOs
  //查询D$是否支持atomiccs操作；
  if (usingRMW) {
    // when xLen < coreDataBits (e.g. RV32D), this AMOALU is wider than necessary
    val amoalu = Module(new AMOALU(coreDataBits))
    amoalu.io.mask := pstore1_mask
    amoalu.io.cmd := (if (usingAtomicsInCache) pstore1_cmd else M_XWR)
    amoalu.io.lhs := s2_data_word
    amoalu.io.rhs := pstore1_data
    pstore1_storegen_data := (if (!usingDataScratchpad) amoalu.io.out else {
      val mask = FillInterleaved(8, Mux(s2_correct, 0.U, pstore1_mask))
      amoalu.io.out_unmasked & mask | s2_data_word_corrected & ~mask
    })
  } else if (!usingAtomics) {
    assert(!(s1_valid_masked && s1_read && s1_write), "unsupported D$ operation")
  }

  // flushes，需要刷新D$时需要做的操作：
  val resetting = RegInit(false.B)
  if (!usingDataScratchpad)
    when (RegNext(reset)) { resetting := true }                                     //1.将重置信号resetting置位；
  val flushCounter = Reg(init=UInt(nSets * (nWays-1), log2Ceil(nSets * nWays)))     
  val flushCounterNext = flushCounter +& 1
  val flushDone = (flushCounterNext >> log2Ceil(nSets)) === nWays
  val flushCounterWrap = flushCounterNext(log2Ceil(nSets)-1, 0)
  ccover(s2_valid_masked && s2_cmd_flush_all && s2_meta_error, "TAG_ECC_ERROR_DURING_FENCE_I", "D$ ECC error in tag array during cache flush")
  ccover(s2_valid_masked && s2_cmd_flush_all && s2_data_error, "DATA_ECC_ERROR_DURING_FENCE_I", "D$ ECC error in data array during cache flush")
  s1_flush_valid := metaArb.io.in(5).fire() && !s1_flush_valid && !s2_flush_valid_pre_tag_ecc && release_state === s_ready && !release_ack_wait
  metaArb.io.in(5).valid := flushing && !flushed                                    //2.查询D$是否可以刷新s1_flush_valid；
  metaArb.io.in(5).bits.write := false                                              //3.屏蔽写信号；
  metaArb.io.in(5).bits.idx := flushCounter(idxBits-1, 0)
  metaArb.io.in(5).bits.addr := Cat(io.cpu.req.bits.addr >> untagBits, metaArb.io.in(5).bits.idx << blockOffBits)
  metaArb.io.in(5).bits.way_en := metaArb.io.in(4).bits.way_en                      //4.根据接收到的来自其他设备传来的数据metaArb.io.in(4)来重新填充D$；
  metaArb.io.in(5).bits.data := metaArb.io.in(4).bits.data

  // Only flush D$ on FENCE.I if some cached executable regions are untracked.
  if (supports_flush) {
    when (s2_valid_masked && s2_cmd_flush_all) {
      when (!flushed && !io.cpu.s2_kill && !release_ack_wait && !uncachedInFlight.asUInt.orR) {
        flushing := true
      }
    }

    when (tl_out_a.fire() && !s2_uncached) { flushed := false }
    when (flushing) {
      s1_victim_way := flushCounter >> log2Up(nSets)                                //s1_victim_way在flush表示将要刷新的line；
      when (s2_flush_valid) {
        flushCounter := flushCounterNext
        when (flushDone) {
          flushed := true
          if (!isPow2(nWays)) flushCounter := flushCounterWrap
        }
      }
      when (flushed && release_state === s_ready && !release_ack_wait) {
        flushing := false
      }
    }
  }
  metaArb.io.in(0).valid := resetting
  metaArb.io.in(0).bits := metaArb.io.in(5).bits
  metaArb.io.in(0).bits.write := true
  metaArb.io.in(0).bits.way_en := ~UInt(0, nWays)
  metaArb.io.in(0).bits.data := tECC.encode(L1Metadata(s2_req.addr >> tagLSB, ClientMetadata.onReset).asUInt)
  when (resetting) {
    flushCounter := flushCounterNext
    when (flushDone) {
      resetting := false
      if (!isPow2(nWays)) flushCounter := flushCounterWrap
    }
  }

  // gate the clock，下面是使能时钟的信号线；
  clock_en_reg := !cacheParams.clockGate ||
    io.ptw.customCSRs.disableDCacheClockGate ||
    io.cpu.keep_clock_enabled ||
    metaArb.io.out.valid || // subsumes resetting || flushing
    s1_probe || s2_probe ||
    s1_valid || s2_valid ||
    pstore1_held || pstore2_valid ||
    release_state =/= s_ready ||
    release_ack_wait || !release_queue_empty ||
    !tlb.io.req.ready ||
    cached_grant_wait || uncachedInFlight.asUInt.orR ||
    lrscCount > 0 || blockProbeAfterGrantCount > 0

  // performance events
  io.cpu.perf.acquire := edge.done(tl_out_a)
  io.cpu.perf.release := edge.done(tl_out_c)
  io.cpu.perf.grant := d_done
  io.cpu.perf.tlbMiss := io.ptw.req.fire()
  io.cpu.perf.storeBufferEmptyAfterLoad := !(
    (s1_valid && s1_write) ||
    ((s2_valid && s2_write && !s2_waw_hazard) || pstore1_held) ||
    pstore2_valid)
  io.cpu.perf.storeBufferEmptyAfterStore := !(
    (s1_valid && s1_write) ||
    (s2_valid && s2_write && pstore1_rmw) ||
    ((s2_valid && s2_write && !s2_waw_hazard || pstore1_held) && pstore2_valid))
  io.cpu.perf.canAcceptStoreThenLoad := !(
    ((s2_valid && s2_write && pstore1_rmw) && (s1_valid && s1_write && !s1_waw_hazard)) ||
    (pstore2_valid && pstore1_valid_likely && (s1_valid && s1_write)))
  io.cpu.perf.canAcceptStoreThenRMW := io.cpu.perf.canAcceptStoreThenLoad && !pstore2_valid
  io.cpu.perf.canAcceptLoadThenLoad := !((s1_valid && s1_write && needsRead(s1_req)) && ((s2_valid && s2_write && !s2_waw_hazard || pstore1_held) || pstore2_valid))
  io.cpu.perf.blocked := {
    // stop reporting blocked just before unblocking to avoid overly conservative stalling
    val beatsBeforeEnd = outer.crossing match {
      case SynchronousCrossing(_) => 2
      case RationalCrossing(_) => 1 // assumes 1 < ratio <= 2; need more bookkeeping for optimal handling of >2
      case _: AsynchronousCrossing => 1 // likewise
    }
    cached_grant_wait && d_address_inc < ((cacheBlockBytes - beatsBeforeEnd * beatBytes) max 0)
  }

  // report errors
  val (data_error, data_error_uncorrectable, data_error_addr) =
    if (usingDataScratchpad) (s2_valid_data_error, s2_data_error_uncorrectable, s2_req.addr) else {
      (RegNext(tl_out_c.fire() && inWriteback && s2_data_error),
        RegNext(s2_data_error_uncorrectable),
        probe_bits.address) // This is stable for a cycle after tl_out_c.fire, so don't need a register
    }
  {
    val error_addr =
      Mux(metaArb.io.in(1).valid, Cat(s2_first_meta_corrected.tag, metaArb.io.in(1).bits.addr(tagLSB-1, idxLSB)),
          data_error_addr >> idxLSB) << idxLSB
    io.errors.uncorrectable.foreach { u =>
      u.valid := metaArb.io.in(1).valid && s2_meta_error_uncorrectable || data_error && data_error_uncorrectable
      u.bits := error_addr
    }
    io.errors.correctable.foreach { c =>
      c.valid := metaArb.io.in(1).valid || data_error
      c.bits := error_addr
      io.errors.uncorrectable.foreach { u => when (u.valid) { c.valid := false } }
    }
    io.errors.bus.valid := tl_out.d.fire() && (tl_out.d.bits.denied || tl_out.d.bits.corrupt)
    io.errors.bus.bits := Mux(grantIsCached, s2_req.addr >> idxLSB << idxLSB, 0.U)

    ccoverNotScratchpad(io.errors.bus.valid && grantIsCached, "D_ERROR_CACHED", "D$ D-channel error, cached")
    ccover(io.errors.bus.valid && !grantIsCached, "D_ERROR_UNCACHED", "D$ D-channel error, uncached")
  }

  if (usingDataScratchpad) {
    val data_error_cover = Seq(
      CoverBoolean(!data_error, Seq("no_data_error")),
      CoverBoolean(data_error && !data_error_uncorrectable, Seq("data_correctable_error")),
      CoverBoolean(data_error && data_error_uncorrectable, Seq("data_uncorrectable_error")))
    val request_source = Seq(
      CoverBoolean(s2_isSlavePortAccess, Seq("from_TL")),
      CoverBoolean(!s2_isSlavePortAccess, Seq("from_CPU")))

    cover(new CrossProperty(
      Seq(data_error_cover, request_source),
      Seq(),
      "MemorySystem;;Scratchpad Memory Bit Flip Cross Covers"))
  } else {

    val data_error_type = Seq(
      CoverBoolean(!s2_valid_data_error, Seq("no_data_error")),
      CoverBoolean(s2_valid_data_error && !s2_data_error_uncorrectable, Seq("data_correctable_error")),
      CoverBoolean(s2_valid_data_error && s2_data_error_uncorrectable, Seq("data_uncorrectable_error")))
    val data_error_dirty = Seq(
      CoverBoolean(!s2_victim_dirty, Seq("data_clean")),
      CoverBoolean(s2_victim_dirty, Seq("data_dirty")))
    val request_source = if (supports_flush) {
        Seq(
          CoverBoolean(!flushing, Seq("access")),
          CoverBoolean(flushing, Seq("during_flush")))
      } else {
        Seq(CoverBoolean(true.B, Seq("never_flush")))
      }
    val tag_error_cover = Seq(
      CoverBoolean( !metaArb.io.in(1).valid, Seq("no_tag_error")),
      CoverBoolean( metaArb.io.in(1).valid && !s2_meta_error_uncorrectable, Seq("tag_correctable_error")),
      CoverBoolean( metaArb.io.in(1).valid && s2_meta_error_uncorrectable, Seq("tag_uncorrectable_error")))
    cover(new CrossProperty(
      Seq(data_error_type, data_error_dirty, request_source, tag_error_cover),
      Seq(),
      "MemorySystem;;Cache Memory Bit Flip Cross Covers"))
  }

  } // leaving gated-clock domain，整个D$中操作结束！farewell；

  def encodeData(x: UInt, poison: Bool) = x.grouped(eccBits).map(dECC.encode(_, if (dECC.canDetect) poison else false.B)).asUInt
  def dummyEncodeData(x: UInt) = x.grouped(eccBits).map(dECC.swizzle(_)).asUInt
  def decodeData(x: UInt) = x.grouped(dECC.width(eccBits)).map(dECC.decode(_))
  def eccMask(byteMask: UInt) = byteMask.grouped(eccBytes).map(_.orR).asUInt      //?
  def eccByteMask(byteMask: UInt) = FillInterleaved(eccBytes, eccMask(byteMask))  //?

  def likelyNeedsRead(req: HellaCacheReq) = {                                     //根据req.cmd初步判断是否为读操作；
    val res = !req.cmd.isOneOf(M_XWR, M_PFW) || req.size < log2Ceil(eccBytes)
    assert(!needsRead(req) || res)
    res
  }
  def needsRead(req: HellaCacheReq) =
    isRead(req.cmd) ||
    (isWrite(req.cmd) && (req.cmd === M_PWR || req.size < log2Ceil(eccBytes)))

  def ccover(cond: Bool, label: String, desc: String)(implicit sourceInfo: SourceInfo) =
    cover(cond, s"DCACHE_$label", "MemorySystem;;" + desc)
  def ccoverNotScratchpad(cond: Bool, label: String, desc: String)(implicit sourceInfo: SourceInfo) =
    if (!usingDataScratchpad) ccover(cond, label, desc)

  require(!usingVM || tagLSB <= pgIdxBits)
  def tagLSB: Int = untagBits
  def probeIdx(b: TLBundleB): UInt = b.address(idxMSB, idxLSB)
  def addressToProbe(vaddr: UInt, paddr: UInt): TLBundleB = {
    val res = Wire(new TLBundleB(edge.bundle))
    res.address := paddr
    res.source := mmioOffset - 1
    res
  }
  def acquire(vaddr: UInt, paddr: UInt, param: UInt): TLBundleA = {
    if (!edge.manager.anySupportAcquireT) Wire(new TLBundleA(edge.bundle))
    else edge.AcquireBlock(UInt(0), paddr >> lgCacheBlockBytes << lgCacheBlockBytes, lgCacheBlockBytes, param)._2
  }

}