
"use strict";

let WaypointPull = require('./WaypointPull.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let ParamPull = require('./ParamPull.js')
let FileTruncate = require('./FileTruncate.js')
let FileMakeDir = require('./FileMakeDir.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let FileOpen = require('./FileOpen.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let CommandBool = require('./CommandBool.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let LogRequestList = require('./LogRequestList.js')
let FileWrite = require('./FileWrite.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let ParamGet = require('./ParamGet.js')
let CommandHome = require('./CommandHome.js')
let CommandTOL = require('./CommandTOL.js')
let StreamRate = require('./StreamRate.js')
let SetMavFrame = require('./SetMavFrame.js')
let FileList = require('./FileList.js')
let FileRemove = require('./FileRemove.js')
let WaypointClear = require('./WaypointClear.js')
let SetMode = require('./SetMode.js')
let FileRename = require('./FileRename.js')
let ParamSet = require('./ParamSet.js')
let WaypointPush = require('./WaypointPush.js')
let CommandLong = require('./CommandLong.js')
let MountConfigure = require('./MountConfigure.js')
let CommandAck = require('./CommandAck.js')
let ParamPush = require('./ParamPush.js')
let LogRequestData = require('./LogRequestData.js')
let FileClose = require('./FileClose.js')
let FileRead = require('./FileRead.js')
let CommandInt = require('./CommandInt.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let FileChecksum = require('./FileChecksum.js')
let MessageInterval = require('./MessageInterval.js')

module.exports = {
  WaypointPull: WaypointPull,
  VehicleInfoGet: VehicleInfoGet,
  ParamPull: ParamPull,
  FileTruncate: FileTruncate,
  FileMakeDir: FileMakeDir,
  CommandTriggerInterval: CommandTriggerInterval,
  FileOpen: FileOpen,
  CommandVtolTransition: CommandVtolTransition,
  CommandBool: CommandBool,
  CommandTriggerControl: CommandTriggerControl,
  LogRequestList: LogRequestList,
  FileWrite: FileWrite,
  WaypointSetCurrent: WaypointSetCurrent,
  FileRemoveDir: FileRemoveDir,
  ParamGet: ParamGet,
  CommandHome: CommandHome,
  CommandTOL: CommandTOL,
  StreamRate: StreamRate,
  SetMavFrame: SetMavFrame,
  FileList: FileList,
  FileRemove: FileRemove,
  WaypointClear: WaypointClear,
  SetMode: SetMode,
  FileRename: FileRename,
  ParamSet: ParamSet,
  WaypointPush: WaypointPush,
  CommandLong: CommandLong,
  MountConfigure: MountConfigure,
  CommandAck: CommandAck,
  ParamPush: ParamPush,
  LogRequestData: LogRequestData,
  FileClose: FileClose,
  FileRead: FileRead,
  CommandInt: CommandInt,
  LogRequestEnd: LogRequestEnd,
  FileChecksum: FileChecksum,
  MessageInterval: MessageInterval,
};
