// See LICENSE.SiFive for license details.

package freechips.rocketchip.subsystem

import Chisel._
import chisel3.internal.sourceinfo.SourceInfo
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.devices.debug.{HasPeripheryDebug, HasPeripheryDebugModuleImp}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.diplomaticobjectmodel.logicaltree._
import freechips.rocketchip.diplomaticobjectmodel.model._
import freechips.rocketchip.tile._

case object HartPrefixKey extends Field[Boolean](false)

// TODO: how specific are these to RocketTiles?
case class TileMasterPortParams(
  buffers: Int = 0,
  cork: Option[Boolean] = None,
  where: TLBusWrapperLocation = SBUS)

case class TileSlavePortParams(
  buffers: Int = 0,
  blockerCtrlAddr: Option[BigInt] = None,
  where: TLBusWrapperLocation = CBUS)

case class RocketCrossingParams(
  crossingType: ClockCrossingType = SynchronousCrossing(),
  master: TileMasterPortParams = TileMasterPortParams(),
  slave: TileSlavePortParams = TileSlavePortParams())

case class RocketAttachParams
  tile: RocketTileParams,
  crossing: RocketCrossingParams,
  lookup: LookupByHartIdImpl
) {
  def instantiate(context: Attachable): RocketTile = {
    val rocket = LazyModule(new RocketTile(tile, crossing, lookup, context.logicalTreeNode))
    rocket
  }
}

case object RocketTilesKey extends Field[Seq[RocketTileParams]](Nil)
case object RocketCrossingKey extends Field[Seq[RocketCrossingParams]](List(RocketCrossingParams()))

trait HasRocketTiles extends HasTiles
    with CanHavePeripheryPLIC
    with CanHavePeripheryCLINT
    with HasPeripheryDebug { this: BaseSubsystem =>
  val module: HasRocketTilesModuleImp

  protected val rocketTileParams = p(RocketTilesKey)
  private val crossings = perTileOrGlobalSetting(p(RocketCrossingKey), rocketTileParams.size)

  // Make a tile and wire its nodes into the system,
  // according to the specified type of clock crossing.
  // Note that we also inject new nodes into the tile itself,
  // also based on the crossing type.
  val rocketTiles = tiles.collect { case r: RocketTile => r }

  rocketTiles.foreach { rocket =>
    connectMasterPortsToSBus(rocket, crossing)
    connectSlavePortsToCBus(rocket, crossing)
    connectInterrupts(rocket, debugOpt, clintOpt, plicOpt)
    LogicalModuleTree.add(logicalTreeNode, rocket.rocketLogicalTree)
  }

  def coreMonitorBundles = (rocketTiles map { t =>
    t.module.core.rocketImpl.coreMonitorBundle
  }).toList
}

trait HasRocketTilesModuleImp extends HasTilesModuleImp
    with HasPeripheryDebugModuleImp {
  val outer: HasRocketTiles
}

// Field for specifying MaskROM addition to subsystem
case object PeripheryMaskROMKey extends Field[Seq[MaskROMParams]](Nil)

class RocketSubsystem(implicit p: Parameters) extends BaseSubsystem
    with HasRocketTiles {
  val tiles = rocketTiles
        
  // add Mask ROM devices
  val maskROMs = p(PeripheryMaskROMKey).map { MaskROM.attach(_, cbus) }

  val hartPrefixNode = if (p(HartPrefixKey)) {
    Some(BundleBroadcast[UInt](registered = true))
  } else {
    None
  }

  val hartPrefixes = hartPrefixNode.map { hpn => Seq.fill(tiles.size) {
   val hps = BundleBridgeSink[UInt]()
   hps := hpn
   hps
  } }.getOrElse(Nil)

  override lazy val module = new RocketSubsystemModuleImp(this)
}

class RocketSubsystemModuleImp[+L <: RocketSubsystem](_outer: L) extends BaseSubsystemModuleImp(_outer)
    with HasResetVectorWire
    with HasRocketTilesModuleImp {

  for (i <- 0 until outer.tiles.size) {
    val wire = tile_inputs(i)
    val prefix = outer.hartPrefixes.lift(i).map(_.bundle).getOrElse(UInt(0))
    wire.hartid := prefix | UInt(outer.hartIdList(i))
    wire.reset_vector := global_reset_vector
  }
}
