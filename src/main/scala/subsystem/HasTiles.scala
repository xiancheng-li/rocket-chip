// See LICENSE.SiFive for license details.

package freechips.rocketchip.subsystem

import Chisel._
import chisel3.dontTouch
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.devices.debug.{HasPeripheryDebug, HasPeripheryDebugModuleImp}
import freechips.rocketchip.devices.tilelink.{BasicBusBlocker, BasicBusBlockerParams, CLINTConsts, PLICKey, CanHavePeripheryPLIC, CanHavePeripheryCLINT}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.diplomaticobjectmodel.logicaltree.{LogicalModuleTree}
import freechips.rocketchip.interrupts._
import freechips.rocketchip.tile.{BaseTile, LookupByHartIdImpl, TileParams, HasExternallyDrivenTileConstants, InstantiatableTileParams}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._

case class TilesLocated(loc: HierarchicalLocation) extends Field[Seq[CanAttachTile]](Nil)

case class TileMasterPortParams(
  buffers: Int = 0,
  cork: Option[Boolean] = None,
  where: TLBusWrapperLocation = SBUS)
{
  def injectNode(context: Attachable)(implicit p: Parameters): TLNode = {
    (TLBuffer(buffers) :=*
      cork.map { u => TLCacheCork(unsafe = u) } .getOrElse { TLTempNode() })
  }
}

case class TileSlavePortParams(
  buffers: Int = 0,
  blockerCtrlAddr: Option[BigInt] = None,
  blockerCtrlWhere: TLBusWrapperLocation = CBUS,
  where: TLBusWrapperLocation = CBUS)
{
  def injectNode(context: Attachable)(implicit p: Parameters): TLNode = {
    val controlBus = context.locateTLBusWrapper(where)
    val blockerBus = context.locateTLBusWrapper(blockerCtrlWhere)
    blockerCtrlAddr
      .map { BasicBusBlockerParams(_, blockerBus.beatBytes, controlBus.beatBytes) }
      .map { bbbp =>
        val blocker = LazyModule(new BasicBusBlocker(bbbp))
        blockerBus.coupleTo("tile_slave_port_bus_blocker") { blocker.controlNode := TLFragmenter(blockerBus) := _ }
        blocker.node
      } .getOrElse { TLTempNode() }
  }
}

trait TileCrossingParamsLike {
  val crossingType: ClockCrossingType
  val master: TileMasterPortParams
  val slave: TileSlavePortParams
}

trait HasTileInterruptSources
  extends CanHavePeripheryPLIC
  with CanHavePeripheryCLINT
  with HasPeripheryDebug
{ this: BaseSubsystem =>
  // define some nodes that are useful for collecting or driving tile interrupts
  val meipNode = p(PLICKey) match {
    case Some(_) => None
    case None    => Some(IntNexusNode(
      sourceFn = { _ => IntSourcePortParameters(Seq(IntSourceParameters(1))) },
      sinkFn   = { _ => IntSinkPortParameters(Seq(IntSinkParameters())) },
      outputRequiresInput = false,
      inputRequiresOutput = false))
  }
}

trait HasTiles extends HasCoreMonitorBundles with HasTileInterruptSources { this: BaseSubsystem =>
  implicit val p: Parameters
  protected val tileAttachParams = p(TilesLocated(location))
  val tiles: Seq[BaseTile] = tileAttachParams.map(_.instantiate(p))
  protected val tileParams: Seq[TileParams] = tiles.map(_.tileParams)
  def nTiles: Int = tileParams.size
  def hartIdList: Seq[Int] = tileParams.map(_.hartId)
  def localIntCounts: Seq[Int] = tileParams.map(_.core.nLocalInterrupts)

  val tileHaltXbarNode = IntXbar(p)
  val tileHaltSinkNode = IntSinkNode(IntSinkPortSimple())
  tileHaltSinkNode := tileHaltXbarNode

  val tileWFIXbarNode = IntXbar(p)
  val tileWFISinkNode = IntSinkNode(IntSinkPortSimple())
  tileWFISinkNode := tileWFIXbarNode

  val tileCeaseXbarNode = IntXbar(p)
  val tileCeaseSinkNode = IntSinkNode(IntSinkPortSimple())
  tileCeaseSinkNode := tileCeaseXbarNode

  tileAttachParams.zip(tiles).foreach { case (params, t) =>
    params.connect(t.asInstanceOf[params.TileType], this)
  }
}

trait CanAttachTile {
  type TileType <: BaseTile
  type TileContextType = Attachable with HasTiles with HasTileInterruptSources
  def tileParams: InstantiatableTileParams[TileType]
  def crossingParams: TileCrossingParamsLike
  def lookup: LookupByHartIdImpl

  def instantiate(implicit p: Parameters): TileType = {
    val tile = LazyModule(tileParams.instantiate(crossingParams, lookup))
    tile
  }

  def connect(tile: TileType, context: TileContextType): Unit = {
    connectMasterPorts(tile, context)
    connectSlavePorts(tile, context)
    connectInterrupts(tile, context)
    LogicalModuleTree.add(context.logicalTreeNode, tile.logicalTreeNode)
  }

  def connectMasterPorts(tile: TileType, context: Attachable): Unit = {
    implicit val p = context.p
    val dataBus = context.locateTLBusWrapper(crossingParams.master.where)
    dataBus.coupleFrom(tileParams.name.getOrElse("tile")) { bus =>
      bus :=* crossingParams.master.injectNode(context) :=* tile.crossMasterPort()
    }
  }

  def connectSlavePorts(tile: TileType, context: Attachable): Unit = {
    implicit val p = context.p
    DisableMonitors { implicit p =>
      val controlBus = context.locateTLBusWrapper(crossingParams.slave.where)
      controlBus.coupleTo(tileParams.name.getOrElse("tile")) { bus =>
        tile.crossSlavePort() :*= crossingParams.slave.injectNode(context) :*= bus
      }
    }
  }

  def connectInterrupts(tile: TileType, context: TileContextType): Unit = {
    implicit val p = context.p
    // Handle all the different types of interrupts crossing to or from the tile:
    // NOTE: The order of calls to := matters! They must match how interrupts
    //       are decoded from tile.intInwardNode inside the tile. For this reason,
    //       we stub out missing interrupts with constant sources here.

    // 1. Debug interrupt is definitely asynchronous in all cases.
    tile.intInwardNode :=
      context.debugOpt
        .map { tile { IntSyncAsyncCrossingSink(3) } := _.intnode }
        .getOrElse { NullIntSource() }

    // 2. The CLINT and PLIC output interrupts are synchronous to the TileLink bus clock,
    //    so might need to be synchronized depending on the Tile's crossing type.

    //    From CLINT: "msip" and "mtip"
    tile.crossIntIn() :=
      context.clintOpt.map { _.intnode }
        .getOrElse { NullIntSource(sources = CLINTConsts.ints) }

    //    From PLIC: "meip"
    tile.crossIntIn() :=
      context.plicOpt .map { _.intnode }
        .getOrElse { context.meipNode.get }

    //    From PLIC: "seip" (only if supervisor mode is enabled)
    if (tile.tileParams.core.hasSupervisorMode) {
      tile.crossIntIn() :=
        context.plicOpt .map { _.intnode }
          .getOrElse { NullIntSource() }
    }

    // 3. Local Interrupts ("lip") are required to already be synchronous to the Tile's clock.
    // (they are connected to tile.intInwardNode in a seperate trait)

    // 4. Interrupts coming out of the tile are sent to the PLIC,
    //    so might need to be synchronized depending on the Tile's crossing type.
    context.plicOpt.foreach { plic =>
      FlipRendering { implicit p =>
        plic.intnode :=* tile.crossIntOut()
      }
    }

    // 5. Reports of tile status are collected without needing to be clock-crossed
    context.tileHaltXbarNode := tile.haltNode
    context.tileWFIXbarNode := tile.wfiNode
    context.tileCeaseXbarNode := tile.ceaseNode
  }
}

trait HasTilesModuleImp extends LazyModuleImp with HasPeripheryDebugModuleImp {
  val outer: HasTiles with HasTileInterruptSources

  def resetVectorBits: Int = {
    // Consider using the minimum over all widths, rather than enforcing homogeneity
    val vectors = outer.tiles.map(_.module.constants.reset_vector)
    require(vectors.tail.forall(_.getWidth == vectors.head.getWidth))
    vectors.head.getWidth
  }

  val tile_inputs = outer.tiles.map(_.module.constants)

  val meip = if(outer.meipNode.isDefined) Some(IO(Vec(outer.meipNode.get.out.size, Bool()).asInput)) else None
  meip.foreach { m =>
    m.zipWithIndex.foreach{ case (pin, i) =>
      (outer.meipNode.get.out(i)._1)(0) := pin
    }
  }
}
