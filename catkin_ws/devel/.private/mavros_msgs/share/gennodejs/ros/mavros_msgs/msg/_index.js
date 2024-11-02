
"use strict";

let HilSensor = require('./HilSensor.js');
let CommandCode = require('./CommandCode.js');
let Waypoint = require('./Waypoint.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let ManualControl = require('./ManualControl.js');
let Thrust = require('./Thrust.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let PositionTarget = require('./PositionTarget.js');
let RCIn = require('./RCIn.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let MountControl = require('./MountControl.js');
let TerrainReport = require('./TerrainReport.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let ActuatorControl = require('./ActuatorControl.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let RCOut = require('./RCOut.js');
let RTKBaseline = require('./RTKBaseline.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let HilControls = require('./HilControls.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let DebugValue = require('./DebugValue.js');
let GPSRAW = require('./GPSRAW.js');
let LandingTarget = require('./LandingTarget.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let Vibration = require('./Vibration.js');
let VehicleInfo = require('./VehicleInfo.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let StatusText = require('./StatusText.js');
let Tunnel = require('./Tunnel.js');
let ESCStatus = require('./ESCStatus.js');
let RTCM = require('./RTCM.js');
let ExtendedState = require('./ExtendedState.js');
let GPSRTK = require('./GPSRTK.js');
let RadioStatus = require('./RadioStatus.js');
let WaypointList = require('./WaypointList.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let VFR_HUD = require('./VFR_HUD.js');
let Mavlink = require('./Mavlink.js');
let State = require('./State.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let LogEntry = require('./LogEntry.js');
let WaypointReached = require('./WaypointReached.js');
let CellularStatus = require('./CellularStatus.js');
let Param = require('./Param.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let SysStatus = require('./SysStatus.js');
let Altitude = require('./Altitude.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let ESCInfo = require('./ESCInfo.js');
let BatteryStatus = require('./BatteryStatus.js');
let LogData = require('./LogData.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let Trajectory = require('./Trajectory.js');
let ParamValue = require('./ParamValue.js');
let HilGPS = require('./HilGPS.js');
let GPSINPUT = require('./GPSINPUT.js');
let HomePosition = require('./HomePosition.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let FileEntry = require('./FileEntry.js');

module.exports = {
  HilSensor: HilSensor,
  CommandCode: CommandCode,
  Waypoint: Waypoint,
  ESCTelemetryItem: ESCTelemetryItem,
  HilStateQuaternion: HilStateQuaternion,
  ManualControl: ManualControl,
  Thrust: Thrust,
  MagnetometerReporter: MagnetometerReporter,
  PositionTarget: PositionTarget,
  RCIn: RCIn,
  OpticalFlowRad: OpticalFlowRad,
  MountControl: MountControl,
  TerrainReport: TerrainReport,
  CamIMUStamp: CamIMUStamp,
  TimesyncStatus: TimesyncStatus,
  ActuatorControl: ActuatorControl,
  OverrideRCIn: OverrideRCIn,
  RCOut: RCOut,
  RTKBaseline: RTKBaseline,
  ESCTelemetry: ESCTelemetry,
  WheelOdomStamped: WheelOdomStamped,
  HilControls: HilControls,
  CompanionProcessStatus: CompanionProcessStatus,
  DebugValue: DebugValue,
  GPSRAW: GPSRAW,
  LandingTarget: LandingTarget,
  GlobalPositionTarget: GlobalPositionTarget,
  Vibration: Vibration,
  VehicleInfo: VehicleInfo,
  AttitudeTarget: AttitudeTarget,
  NavControllerOutput: NavControllerOutput,
  ESCInfoItem: ESCInfoItem,
  StatusText: StatusText,
  Tunnel: Tunnel,
  ESCStatus: ESCStatus,
  RTCM: RTCM,
  ExtendedState: ExtendedState,
  GPSRTK: GPSRTK,
  RadioStatus: RadioStatus,
  WaypointList: WaypointList,
  EstimatorStatus: EstimatorStatus,
  VFR_HUD: VFR_HUD,
  Mavlink: Mavlink,
  State: State,
  PlayTuneV2: PlayTuneV2,
  LogEntry: LogEntry,
  WaypointReached: WaypointReached,
  CellularStatus: CellularStatus,
  Param: Param,
  OnboardComputerStatus: OnboardComputerStatus,
  SysStatus: SysStatus,
  Altitude: Altitude,
  CameraImageCaptured: CameraImageCaptured,
  ESCInfo: ESCInfo,
  BatteryStatus: BatteryStatus,
  LogData: LogData,
  HilActuatorControls: HilActuatorControls,
  Trajectory: Trajectory,
  ParamValue: ParamValue,
  HilGPS: HilGPS,
  GPSINPUT: GPSINPUT,
  HomePosition: HomePosition,
  ESCStatusItem: ESCStatusItem,
  ADSBVehicle: ADSBVehicle,
  FileEntry: FileEntry,
};
