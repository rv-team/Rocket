
/*
  Edit Author : wn
  Date        : 2019.8.12
  */
// See LICENSE.Berkeley for license details.

package freechips.rocketchip.rocket

import Chisel._
import scala.collection.mutable.{ArrayBuffer, Map}

object DecodeLogic
{
  def term(lit: BitPat) =  //每次调用term都是实例化Term,term方法把BitPat，转换为Term
    new Term(lit.value, BigInt(2).pow(lit.getWidth)-(lit.mask+1))  // Term的值即为BitPat的值,Term的mask为BitPat的mask取反
    //mask补码 = 2^n - mask = ~mask + 1 即：~mask = 2^n - mask - 1 = 2^n - (mask + 1)
  def logic(addr: UInt, addrWidth: Int, cache: Map[Term,Bool], terms: Seq[Term]) = {  //把addr与terms中的所有项逐个比对，返回比对结果。
    terms.map { t =>
      cache.getOrElseUpdate(t, (if (t.mask == 0) addr else addr & Bits(BigInt(2).pow(addrWidth)-(t.mask+1), addrWidth)) === Bits(t.value, addrWidth))
    }.foldLeft(Bool(false))(_||_)       //若每bit都关心返回原指令，否则将不关心bit化成0 //Bits（~t.mask，addrWidth)
  } //getOrElseUpdate返回的是队列吗？

  //应该是解码一个信号的
  //default: BitPat是def default: List[BitPat]中第i位信号；
  // mapping: Iterable[(BitPat, BitPat)]中每个元素是  key -> （第i信号)value
	def apply(addr: UInt, default: BitPat, mapping: Iterable[(BitPat, BitPat)]): UInt = { //addr即是inst，也就是指令本身，是不是key？
    val cache = caches.getOrElseUpdate(addr, Map[Term,Bool]())  //在caches里查找，命中或更新
    val dterm = term(default)           //调用term方法把default转换为项（Term类的实例，如最小项或最大项）

    //keys, values，先分别转成term类，再合在一起
    val (keys, values) = mapping.unzip  //得到两个列表，（原第一个元素自成一列表，第二个元素）把映射表mapping解为一组keys和一组values
    val addrWidth = keys.map(_.getWidth).max //取元素位宽最大值 求出keys的最大宽度
    val terms = keys.toList.map(k => term(k))    //把keys转变成为Term
    val termvalues = terms zip values.toList.map(term(_))  //每一项变成term的  mapping

    for (t <- keys.zip(terms).tails; if !t.isEmpty)  // 检查terms是否存在重叠的情况，即各个指令的编码是否存在重叠的情况
      for (u <- t.tail)
        assert(!t.head._2.intersects(u._2), "DecodeLogic: keys " + t.head + " and " + u + " overlap")


// 逐个解出信号value的每一位：(0 until vMaxWidth).map(...)，然后组合成为所对应的信号
//mint即minTerms，即信号的第i位为1的key的集合，也就是指令的集合。
//maxt即maxTerms，即信号的第i为为0的key的集合，也是指令的集合。
//k对应的为key，即指令编码
//t对应的为要解码的信号
//
    Cat((0 until default.getWidth.max(values.map(_.getWidth).max)).map({ case (i: Int) =>
      val mint = termvalues.filter { case (k,t) => ((t.mask >> i) & 1) == 0 && ((t.value >> i) & 1) == 1 }.map(_._1) //最小项，取出关心项中对应值为1的key
      val maxt = termvalues.filter { case (k,t) => ((t.mask >> i) & 1) == 0 && ((t.value >> i) & 1) == 0 }.map(_._1)  //最大项，取出关心项中对应值为0的
      val dc = termvalues.filter { case (k,t) => ((t.mask >> i) & 1) == 1 }.map(_._1)   //取出无关项，mask为1的变量为无关项

      if (((dterm.mask >> i) & 1) != 0) {  //默认项mask=0,即关心位
        logic(addr, addrWidth, cache, SimplifyDC(mint, maxt, addrWidth))
      } else {
        val defbit = (dterm.value.toInt >> i) & 1  //default第i位对应值
        val t = if (defbit == 0) mint else maxt  //最小项或最大项
        val bit = logic(addr, addrWidth, cache, Simplify(t, dc, addrWidth)) //bit为查找结果，找到为1，找不到为0,
        if (defbit == 0) bit else ~bit
      }
    }).reverse)  //因为前者L SB在前，后者MSB在前，所以需要使用reverse逆序
  }

  //mappingIn即为decode_table，其中为key（BitPat）到values（Seq[BitPat]）的映射；
  //mapping(i)为key（BitPat）到values中第i个BitPat（如：rocc, br, jal等）的映射表；
  //调用另外一个apply，每次解出一个value，即每次生成解出一个value的逻辑：
  def apply(addr: UInt, default: Seq[BitPat], mappingIn: Iterable[(BitPat, Seq[BitPat])]): Seq[UInt] = {
    val mapping = ArrayBuffer.fill(default.size)(ArrayBuffer[(BitPat, BitPat)]())  //建立 位宽个pair的 数组
    for ((key, values) <- mappingIn)
      for ((value, i) <- values zipWithIndex)
        mapping(i) += key -> value  //一个key对应很多1bit的value.但是有很多key，mapping是有default.size 个行，每一行都是ArrayBuffer[(BitPat, BitPat)]()
    for ((thisDefault, thisMapping) <- default zip mapping) //thisDefault是BitPat,thisMapping是ArrayBuffer[(BitPat, BitPat)](mappingIn.size)
      yield apply(addr, thisDefault, thisMapping)  //调用L18的apply
  }
  def apply(addr: UInt, default: Seq[BitPat], mappingIn: List[(UInt, Seq[BitPat])]): Seq[UInt] =
    apply(addr, default, mappingIn.map(m => (BitPat(m._1), m._2)).asInstanceOf[Iterable[(BitPat, Seq[BitPat])]])
  def apply(addr: UInt, trues: Iterable[UInt], falses: Iterable[UInt]): Bool =
    apply(addr, BitPat.dontCare(1), trues.map(BitPat(_) -> BitPat("b1")) ++ falses.map(BitPat(_) -> BitPat("b0"))).asBool
  private val caches = Map[UInt,Map[Term,Bool]]()  //定义空的caches，用到时会更新，
}

class Term(val value: BigInt, val mask: BigInt = 0)  //定义了Term可以使用别的一些方法，
{
  var prime = true

  def covers(x: Term) = ((value ^ x.value) & ~ mask | x.mask &~ mask) == 0 //this和x关心的位都相同，且关心位的value都相同。 
  def intersects(x: Term) = ((value ^ x.value) &~ mask &~ x.mask) == 0  //存在 care位上的value相同 即可
  override def equals(that: Any) = that match {
    case x: Term => x.value == value && x.mask == mask  //care的位都相同，care位和 不care位 的value都相同。
    case _ => false
  }
  override def hashCode = value.toInt
  def < (that: Term) = value < that.value || value == that.value && mask < that.mask  //定义在该表中可以用的函数或方法
  def similar(x: Term) = {
    val diff = value - x.value
    mask == x.mask && value > x.value && (diff & diff-1) == 0  //关心的位相同，有且只有一个变量不同
  }  //两个的mask相同，前值大于后值，仅有一个变量不同
  def merge(x: Term) = {    //(i.j+1)合并(i,j)
    prime = false  //省略另外that,that.prime
    x.prime = false
    val bit = value - x.value  

    new Term(value &~ bit, mask | bit)  //该位置0，并将该位的mask置1，表示不care此位。Term中的mask是补码形式存在。
  }

  override def toString = value.toString(16) + "-" + mask.toString(16) + (if (prime) "p" else "")
}

object Simplify
{
  //获取 化简 蕴含项
  def getPrimeImplicants(implicants: Seq[Term], bits: Int) = {  // bits：最小项中变量的个数
    var prime = List[Term]()  //蕴涵项
    implicants.foreach(_.prime = true)
    val cols = (0 to bits).map(b => implicants.filter(b == _.mask.bitCount))   //关心几个位就用几位变量表示                                                                                  ))  //位掩码与bit位一致
    val table = cols.map(c => (0 to bits).map(b => collection.mutable.Set(c.filter(b == _.value.bitCount):_*))) //值中为1的个数与变量个数相同
    for (i <- 0 to bits) {
      for (j <- 0 until bits-i)
        table(i)(j).foreach(a => table(i+1)(j) ++= table(i)(j+1).filter(_.similar(a)).map(_.merge(a))) //尝试把单元格(i,j)中未合并的质项进行化简
      for (r <- table(i))
        for (p <- r; if p.prime)
          prime = p :: prime
    }
    prime.sortWith(_<_) //排序
  }
  //获取质 蕴涵项
  def getEssentialPrimeImplicants(prime: Seq[Term], minterms: Seq[Term]): (Seq[Term],Seq[Term],Seq[Term]) = {
    val primeCovers = prime.map(p => minterms.filter(p covers _))  //质项 覆盖的最小项
    for (((icover, pi), i) <- (primeCovers zip prime).zipWithIndex) {  //list[（（覆盖的最小项），原项）]
      for (((jcover, pj), j) <- (primeCovers zip prime).zipWithIndex.drop(i+1)) {
        if (icover.size > jcover.size && jcover.forall(pi covers _))  //pj是pi的子集
          return getEssentialPrimeImplicants(prime.filter(_ != pj), minterms)  //去掉子集pj
      }
    }
//此时prime是 质蕴涵项
    val essentiallyCovered = minterms.filter(t => prime.count(_ covers t) == 1) //只出现过一次的最小项。对于t最小项，只有一个项包含它；该项为必要质蕴涵项
    val essential = prime.filter(p => essentiallyCovered.exists(p covers _)) //必要质蕴含项
    val nonessential = prime.filterNot(essential contains _)  //非 必要质蕴含项
    val uncovered = minterms.filterNot(t => essential.exists(_ covers t)) //去掉 必要质蕴含项包含的最小项；即 多次出现的最小项
    if (essential.isEmpty || uncovered.isEmpty) //若必要质蕴含项为空，或者必要质蕴涵项覆盖了所有最小项
      (essential, nonessential, uncovered)  //返回队列
    else {
      val (a, b, c) = getEssentialPrimeImplicants(nonessential, uncovered)  //循环操作，直到覆盖所有最小项；所以返回的a队列已经覆盖所有项了
      (essential ++ a, b, c)
    }
  }
  def getCost(cover: Seq[Term], bits: Int) = cover.map(bits - _.mask.bitCount).sum
  def cheaper(a: List[Term], b: List[Term], bits: Int) = {
    val ca = getCost(a, bits)
    val cb = getCost(b, bits)
    def listLess(a: List[Term], b: List[Term]): Boolean = !b.isEmpty && (a.isEmpty || a.head < b.head || a.head == b.head && listLess(a.tail, b.tail))
    ca < cb || ca == cb && listLess(a.sortWith(_<_), b.sortWith(_<_))
  }
  def getCover(implicants: Seq[Term], minterms: Seq[Term], bits: Int) = {
    if (minterms.nonEmpty) {
      val cover = minterms.map(m => implicants.filter(_.covers(m)))
      val all = cover.tail.foldLeft(cover.head.map(Set(_)))((c0, c1) => c0.flatMap(a => c1.map(a + _)))
      all.map(_.toList).reduceLeft((a, b) => if (cheaper(a, b, bits)) a else b)
    } else
      Seq[Term]()
  }
  def stringify(s: Seq[Term], bits: Int) = s.map(t => (0 until bits).map(i => if ((t.mask & (1 << i)) != 0) "x" else ((t.value >> i) & 1).toString).reduceLeft(_+_).reverse).reduceLeft(_+" + "+_)

  def apply(minterms: Seq[Term], dontcares: Seq[Term], bits: Int) = {
    val prime = getPrimeImplicants(minterms ++ dontcares, bits)
    minterms.foreach(t => assert(prime.exists(_.covers(t))))
    val (eprime, prime2, uncovered) = getEssentialPrimeImplicants(prime, minterms)
    val cover = eprime ++ getCover(prime2, uncovered, bits)
    minterms.foreach(t => assert(cover.exists(_.covers(t)))) // sanity check
    cover
  }
}

object SimplifyDC  //SimplifyDC使用类似卡诺图化简的方法简化解码逻辑，返回Seq[Term]。logic中生成逐个比较并返回的逻辑
{
  //获取蕴含项
  def getImplicitDC(maxterms: Seq[Term], term: Term, bits: Int, above: Boolean): Term = {
    for (i <- 0 until bits) {
      var t: Term = null
      if (above && ((term.value | term.mask) & (BigInt(1) << i)) == 0)  //value=0，且mask=0；
        t = new Term(term.value | (BigInt(1) << i), term.mask)  //新建一个t,term.value原=0.现改成1，条件： mask第i位为0
      else if (!above && (term.value & (BigInt(1) << i)) != 0)  //value第i位=1
        t = new Term(term.value & ~(BigInt(1) << i), term.mask)  //term.value原第i位=1.现改成0，那更改的条件是什么？ 
      if (t != null && !maxterms.exists(_.intersects(t)))
        return t
    }
    null
  }
  //获取质蕴涵项
  def getPrimeImplicants(minterms: Seq[Term], maxterms: Seq[Term], bits: Int) = {
    var prime = List[Term]()
    minterms.foreach(_.prime = true) //？？
    var mint = minterms.map(t => new Term(t.value, t.mask))
    val cols = (0 to bits).map(b => mint.filter(b == _.mask.bitCount))
    val table = cols.map(c => (0 to bits).map(b => collection.mutable.Set(c.filter(b == _.value.bitCount):_*)))

    for (i <- 0 to bits) {  //i是列，j是行
      for (j <- 0 until bits-i) {
        table(i)(j).foreach(a => table(i+1)(j) ++= table(i)(j+1).filter(_ similar a).map(_ merge a))
        //单元格(i,j+1)中的质项，合并吸收一个变量之后，value中1的位数减少一个，而mask中1的位数增加一个，所以也是添加到单元格(i+1,j)中。
      }
      for (j <- 0 until bits-i) {
        for (a <- table(i)(j).filter(_.prime)) {  //去掉空的？
          val dc = getImplicitDC(maxterms, a, bits, true)
          if (dc != null)
            table(i+1)(j) += dc merge a
        }
        for (a <- table(i)(j+1).filter(_.prime)) {
          val dc = getImplicitDC(maxterms, a, bits, false)
          if (dc != null)
            table(i+1)(j) += a merge dc
        }
      }
      for (r <- table(i))
        for (p <- r; if p.prime)
          prime = p :: prime
    }
    prime.sortWith(_<_)
  }

  def verify(cover: Seq[Term], minterms: Seq[Term], maxterms: Seq[Term]) = {  //验证
    assert(minterms.forall(t => cover.exists(_ covers t)))
    assert(maxterms.forall(t => !cover.exists(_ intersects t)))
  }
  def apply(minterms: Seq[Term], maxterms: Seq[Term], bits: Int) = {  //最后调用之前函数，由最小项和最大项得出最简输出。
    val prime = getPrimeImplicants(minterms, maxterms, bits)
    val (eprime, prime2, uncovered) = Simplify.getEssentialPrimeImplicants(prime, minterms)
    val cover = eprime ++ Simplify.getCover(prime2, uncovered, bits)
    verify(cover, minterms, maxterms)
    cover
  }
}
