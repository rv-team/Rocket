// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

/*Hella cache是ROCKET的非阻塞cache，专为顺序处理器设计；*/
package freechips.rocketchip.rocket

import Chisel._
import chisel3.experimental.dontTouch
import freechips.rocketchip.config.{Parameters, Field}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import scala.collection.mutable.ListBuffer

/*DCacheParams：定义该类DCache需要的全部参数；
 *hellacache为4路64组关联cache，一行占64b，其中数据占1B，32个TLB入口,每个block占据64b；
 *nMSHRs：配置一个MISS STATUS HOLDING REG,只存在于DCache中;
 *nSDQ(store data queue):可以存放17个store请求；
 *nRPQ(replay queue)；
 *nMMIOs(memory map I/O)：针对uncached的外设通过MMIO来访问I/O Space；
 *separateUncachedResp：访问IO Space时，该信号有效；
 *acquireBeforeRelease：在释放D$中某共享的参数之前，需要确保该参数的使用结束后再进行释放任务；
 *pipelineWayMux：该信号决定s1处理cached data/s1s2处理uncached data；
 *scratchpad provides low-latency high-thrpughput temporary storage;
*/
case class DCacheParams(
    nSets: Int = 64,
    nWays: Int = 4,
    rowBits: Int = 64,
    nTLBEntries: Int = 32,
    tagECC: Option[String] = None,
    dataECC: Option[String] = None,
    dataECCBytes: Int = 1,
    nMSHRs: Int = 1,
    nSDQ: Int = 17,
    nRPQ: Int = 16,
    nMMIOs: Int = 1,
    blockBytes: Int = 64,
    separateUncachedResp: Boolean = false,
    acquireBeforeRelease: Boolean = false,
    pipelineWayMux: Boolean = false,
    clockGate: Boolean = false,
    scratch: Option[BigInt] = None) extends L1CacheParams {
//tagCode/dataCode：纠错后的tag/data；
  def tagCode: Code = Code.fromString(tagECC)
  def dataCode: Code = Code.fromString(dataECC)
//rocket使用了SPM；
  def dataScratchpadBytes: Int = scratch.map(_ => nSets*blockBytes).getOrElse(0)
//选择random作为替换原则；
  def replacement = new RandomReplacement(nWays)

  require((!scratch.isDefined || nWays == 1),
    "Scratchpad only allowed in direct-mapped cache.")
  require((!scratch.isDefined || nMSHRs == 0),
    "Scratchpad only allowed in blocking cache.")
  if (scratch.isEmpty)
    require(isPow2(nSets), s"nSets($nSets) must be pow2")
}
/*HasL1HellaCacheParameters：定义D$常用参数性质；
*/
trait HasL1HellaCacheParameters extends HasL1CacheParameters with HasCoreParameters {
  val cacheParams = tileParams.dcache.get
  val cfg = cacheParams
//定义$对应字节大小；
  def wordBits = coreDataBits
  def wordBytes = coreDataBytes
  def wordOffBits = log2Up(wordBytes)
  def beatBytes = cacheBlockBytes / cacheDataBeats
  def beatWords = beatBytes / wordBytes
  def beatOffBits = log2Up(beatBytes)
  def idxMSB = untagBits-1
  def idxLSB = blockOffBits
  def offsetmsb = idxLSB-1
  def offsetlsb = wordOffBits
  def rowWords = rowBits/wordBits
  def doNarrowRead = coreDataBits * nWays % rowBits == 0
  def eccBytes = cacheParams.dataECCBytes
  val eccBits = cacheParams.dataECCBytes * 8
  //enc是对$中纠错后的datacode处理；
  val encBits = cacheParams.dataCode.width(eccBits)
  val encWordBits = encBits * (wordBits / eccBits)
  def encDataBits = cacheParams.dataCode.width(coreDataBits) // NBDCache only
  def encRowBits = encDataBits*rowWords
  //LR/SC: Support for atomic instructions is divided into two categories: LR/SC and AMOs.
  def lrscCycles = coreParams.lrscCycles // ISA requires 16-insn LRSC sequences to succeed
  def lrscBackoff = 3 // disallow LRSC reacquisition briefly
  def blockProbeAfterGrantCycles = 8 // give the processor some time to issue a request after a grant
  //D$中可处理uncached map数目；
  def nIOMSHRs = cacheParams.nMMIOs
  def maxUncachedInFlight = cacheParams.nMMIOs
  def dataScratchpadSize = cacheParams.dataScratchpadBytes
  //检查rowbits是否大于coreDataBits；
  require(rowBits >= coreDataBits, s"rowBits($rowBits) < coreDataBits($coreDataBits)")
  if (!usingDataScratchpad)
    require(rowBits == cacheDataBits, s"rowBits($rowBits) != cacheDataBits($cacheDataBits)")
  // would need offset addr for puts if data width < xlen
  require(xLen <= cacheDataBits, s"xLen($xLen) > cacheDataBits($cacheDataBits)")
}

abstract class L1HellaCacheModule(implicit val p: Parameters) extends Module
  with HasL1HellaCacheParameters

abstract class L1HellaCacheBundle(implicit val p: Parameters) extends ParameterizedBundle()(p)
  with HasL1HellaCacheParameters

/*HasCoreMemOp：Bundle definitions for HellaCache interfaces 
 *cmd：对应load/store指令；
 *size:D$中data大小；
*/
trait HasCoreMemOp extends HasCoreParameters {
  val addr = UInt(width = coreMaxAddrBits)
  val tag  = Bits(width = dcacheReqTagBits)
  val cmd  = Bits(width = M_SZ)
  val size = Bits(width = log2Ceil(coreDataBytes.log2 + 1))
  val signed = Bool()
}

/*HasCoreData：封装data；*/
trait HasCoreData extends HasCoreParameters {
  val data = Bits(width = coreDataBits)
}
/*HellaCacheReqInternal：封装$内部请求信息；*/
class HellaCacheReqInternal(implicit p: Parameters) extends CoreBundle()(p) with HasCoreMemOp {
  val phys = Bool()
  val no_alloc = Bool()
}

/*HellaCacheReq：定义整个$模块的请求模块；*/
class HellaCacheReq(implicit p: Parameters) extends HellaCacheReqInternal()(p) with HasCoreData

/*HellaCacheResp：定义整个$模块的响应模块；
 *replay：回滚标志位，用于cache miss时从内存中重装载对应data；
 *has_data：对应处理read指令时，内存中是否有相应的data被读出；
 *data_word_bypass：处理uncached map时，需要从旁路来访问对应内存地址；
 *data_raw：D$输出的data；
 *store_data：对应处理store指令，存储data至内存；
*/
class HellaCacheResp(implicit p: Parameters) extends CoreBundle()(p)
    with HasCoreMemOp
    with HasCoreData {
  val replay = Bool()
  val has_data = Bool()
  val data_word_bypass = Bits(width = coreDataBits)
  val data_raw = Bits(width = coreDataBits)
  val store_data = Bits(width = coreDataBits)
}

class AlignmentExceptions extends Bundle {
  val ld = Bool()
  val st = Bool()
}
/*HellaCacheExceptions：针对load/store指令的指令预取/地址扩展地址对齐问题而引发的异常；*/
class HellaCacheExceptions extends Bundle {
  val ma = new AlignmentExceptions
  val pf = new AlignmentExceptions
  val ae = new AlignmentExceptions
}
/*HellaCacheWriteData：对D$写数据；*/
class HellaCacheWriteData(implicit p: Parameters) extends CoreBundle()(p) {
  val data = UInt(width = coreDataBits)
  val mask = UInt(width = coreDataBytes)
}
/*HellaCachePerfEvents:记录$的工作情况；
 *acquire/release：读取/释放data状态；
 *grant：$得到授权后才可以获取data；
 *blocked：标记D$的A/B/C/D channel是否处于被阻塞状态（待后期研究）；
 *canAcceptStoreThenRMW：store后load及memory write(io.cpu.perf.canAcceptStoreThenRMW := io.cpu.perf.canAcceptStoreThenLoad && !pstore2_valid);
 *storeBufferEmptyAfterLoad/storeBufferEmptyAfterStore:针对store buffer是否为空的记录；
*/
class HellaCachePerfEvents extends Bundle {
  val acquire = Bool()
  val release = Bool()
  val grant = Bool()
  val tlbMiss = Bool()
  val blocked = Bool()
  val canAcceptStoreThenLoad = Bool()
  val canAcceptStoreThenRMW = Bool()
  val canAcceptLoadThenLoad = Bool()
  val storeBufferEmptyAfterLoad = Bool()
  val storeBufferEmptyAfterStore = Bool()
}

// interface between D$ and processor/DTLB
class HellaCacheIO(implicit p: Parameters) extends CoreBundle()(p) {
  val req = Decoupled(new HellaCacheReq)
  val s1_kill = Bool(OUTPUT) // kill previous cycle's req
  val s1_data = new HellaCacheWriteData().asOutput // data for previous cycle's req
  val s2_nack = Bool(INPUT) // req from two cycles ago is rejected
  val s2_nack_cause_raw = Bool(INPUT) // reason for nack is store-load RAW hazard (performance hint)
  val s2_kill = Bool(OUTPUT) // kill req from two cycles ago
  val s2_uncached = Bool(INPUT) // advisory signal that the access is MMIO
  val s2_paddr = UInt(INPUT, paddrBits) // translated address

  val resp = Valid(new HellaCacheResp).flip  //响应外部read请求后的信号；
  val replay_next = Bool(INPUT)  //重新加载读取/写data操作；
  val s2_xcpt = (new HellaCacheExceptions).asInput
  val uncached_resp = tileParams.dcache.get.separateUncachedResp.option(Decoupled(new HellaCacheResp).flip)
  val ordered = Bool(INPUT)  //表示该D$处于准备状态；
  val perf = new HellaCachePerfEvents().asInput

  val keep_clock_enabled = Bool(OUTPUT) // should D$ avoid clock-gating itself?
  val clock_enabled = Bool(INPUT) // is D$ currently being clocked?
}

/* Base classes for Diplomatic TL2 HellaCaches */
//整个rocket core是采用tile-link互联方式，每个core都是一个tile，有共享资源(关于这种互联方式的优势还需研究)；
abstract class HellaCache(hartid: Int)(implicit p: Parameters) extends LazyModule {
  private val tileParams = p(TileKey)
  protected val cfg = tileParams.dcache.get
  
  protected def cacheClientParameters = cfg.scratch.map(x => Seq()).getOrElse(Seq(TLClientParameters(
    name          = s"Core ${hartid} DCache",
    sourceId      = IdRange(0, 1 max cfg.nMSHRs),
    supportsProbe = TransferSizes(cfg.blockBytes, cfg.blockBytes))))

  protected def mmioClientParameters = Seq(TLClientParameters(
    name          = s"Core ${hartid} DCache MMIO",
    sourceId      = IdRange(firstMMIO, firstMMIO + cfg.nMMIOs),
    requestFifo   = true))

  def firstMMIO = (cacheClientParameters.map(_.sourceId.end) :+ 0).max
  //当前D$在TL中的节点位置；
  val node = TLClientNode(Seq(TLClientPortParameters(
    cacheClientParameters ++ mmioClientParameters,
    minLatency = 1)))

  val module: HellaCacheModule
  //flushOnFenceI：刷新D$<-scratch mem空且未出现supportsAcquireT/executable/TRACKED/IDEMPOTENT模式；
  def flushOnFenceI = cfg.scratch.isEmpty && !node.edges.out(0).manager.managers.forall(m => !m.supportsAcquireT || !m.executable || m.regionType >= RegionType.TRACKED || m.regionType <= RegionType.IDEMPOTENT)

  require(!tileParams.core.haveCFlush || cfg.scratch.isEmpty, "CFLUSH_D_L1 instruction requires a D$")
}

class HellaCacheBundle(val outer: HellaCache)(implicit p: Parameters) extends CoreBundle()(p) {
  val hartid = UInt(INPUT, hartIdLen)
  val cpu = (new HellaCacheIO).flip
  val ptw = new TLBPTWIO()
  val errors = new DCacheErrors
}
/*HellaCacheModule：当前$模式定义；
 *edge：D$工作的触发条件；
 *io：包含对D$发出req的对应cpu信息，所需PTW以及$出错后处理端口；
*/
class HellaCacheModule(outer: HellaCache) extends LazyModuleImp(outer)
    with HasL1HellaCacheParameters {
  implicit val edge = outer.node.edges.out(0)
  val (tl_out, _) = outer.node.out(0)
  val io = IO(new HellaCacheBundle(outer))
  dontTouch(io.cpu.resp) // Users like to monitor these fields even if the core ignores some signals
  dontTouch(io.cpu.s1_data)
  //IOMSHR为FIFO形式；
  private val fifoManagers = edge.manager.managers.filter(TLFIFOFixer.allVolatile)
  fifoManagers.foreach { m =>
    require (m.fifoId == fifoManagers.head.fifoId,
      s"IOMSHRs must be FIFO for all regions with effects, but HellaCache sees\n"+
      s"${m.nodePath.map(_.name)}\nversus\n${fifoManagers.head.nodePath.map(_.name)}")
  }
}

/** Mix-ins for constructing tiles that have a HellaCache */
//HasHellaCache：定义使用HELLACACHE的基本信息：端口号/lazymodule/是否为uncached map；
trait HasHellaCache { this: BaseTile =>
  val module: HasHellaCacheModule
  implicit val p: Parameters
  var nDCachePorts = 0
  lazy val dcache: HellaCache = LazyModule(
    if(tileParams.dcache.get.nMSHRs == 0) {
      new DCache(hartId, crossing)
    } else { new NonBlockingDCache(hartId) })

  tlMasterXbar.node := dcache.node
}

trait HasHellaCacheModule {
  val outer: HasHellaCache with HasTileParameters
  implicit val p: Parameters
  val dcachePorts = ListBuffer[HellaCacheIO]()
  val dcacheArb = Module(new HellaCacheArbiter(outer.nDCachePorts)(outer.p)) //涉及到仲裁；
  outer.dcache.module.io.cpu <> dcacheArb.io.mem
}

/** Metadata array used for all HellaCaches */
//L1Metadata：包含tag信息，以及获取cpu的req信号以及输出resp信号；
class L1Metadata(implicit p: Parameters) extends L1HellaCacheBundle()(p) {
  val coh = new ClientMetadata
  val tag = UInt(width = tagBits)
}

object L1Metadata {
  def apply(tag: Bits, coh: ClientMetadata)(implicit p: Parameters) = {
    val meta = Wire(new L1Metadata)
    meta.tag := tag
    meta.coh := coh  //？
    meta
  }
}
/*L1MetaReadReq：meta响应read；
 *idx：索引set；
 *way_en：选择指定set中的对应block；
*/
class L1MetaReadReq(implicit p: Parameters) extends L1HellaCacheBundle()(p) {
  val idx    = UInt(width = idxBits)
  val way_en = UInt(width = nWays)
  val tag    = UInt(width = tagBits)
}
/*L1MetaWriteReq:meta响应write；
 *data:需要写入memory的data；
*/
class L1MetaWriteReq(implicit p: Parameters) extends L1MetaReadReq()(p) {
  val data = new L1Metadata
}
/*L1MetadataArray：真正地meta机制；
*/
// <: It means an abstract type member is defined (inside some context, e.g. a trait or class), so that concrete implementations of that context must define that type. However, there is a constraint that this type (Currency) must actually be a subtype of AbstractCurrency. 
class L1MetadataArray[T <: L1Metadata](onReset: () => T)(implicit p: Parameters) extends L1HellaCacheModule()(p) {
  val rstVal = onReset()
  val io = new Bundle {
    val read = Decoupled(new L1MetaReadReq).flip
    val write = Decoupled(new L1MetaWriteReq).flip
    val resp = Vec(nWays, rstVal.cloneType).asOutput
  }
  val rst_cnt = Reg(init=UInt(0, log2Up(nSets+1)))
  val rst = rst_cnt < UInt(nSets)
  val waddr = Mux(rst, rst_cnt, io.write.bits.idx)
  val wdata = Mux(rst, rstVal, io.write.bits.data).asUInt
  val wmask = Mux(rst || Bool(nWays == 1), SInt(-1), io.write.bits.way_en.asSInt).asBools
  val rmask = Mux(rst || Bool(nWays == 1), SInt(-1), io.read.bits.way_en.asSInt).asBools
  when (rst) { rst_cnt := rst_cnt+UInt(1) }

  val metabits = rstVal.getWidth
  val tag_array = SeqMem(nSets, Vec(nWays, UInt(width = metabits)))
  val wen = rst || io.write.valid
  when (wen) {
    tag_array.write(waddr, Vec.fill(nWays)(wdata), wmask)
  }
  io.resp := tag_array.read(io.read.bits.idx, io.read.fire()).map(rstVal.fromBits(_))

  io.read.ready := !wen // so really this could be a 6T RAM
  io.write.ready := !rst
}
