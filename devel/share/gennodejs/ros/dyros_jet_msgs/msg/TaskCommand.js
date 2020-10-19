// Auto-generated. Do not edit!

// (in-package dyros_jet_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TaskCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.end_effector = null;
      this.mode = null;
      this.pose = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('end_effector')) {
        this.end_effector = initObj.end_effector
      }
      else {
        this.end_effector = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new Array(4).fill(new geometry_msgs.msg.Pose());
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TaskCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [end_effector] has the right length
    if (obj.end_effector.length !== 4) {
      throw new Error('Unable to serialize array field end_effector - length must be 4')
    }
    // Serialize message field [end_effector]
    bufferOffset = _arraySerializer.bool(obj.end_effector, buffer, bufferOffset, 4);
    // Check that the constant length array field [mode] has the right length
    if (obj.mode.length !== 4) {
      throw new Error('Unable to serialize array field mode - length must be 4')
    }
    // Serialize message field [mode]
    bufferOffset = _arraySerializer.uint32(obj.mode, buffer, bufferOffset, 4);
    // Check that the constant length array field [pose] has the right length
    if (obj.pose.length !== 4) {
      throw new Error('Unable to serialize array field pose - length must be 4')
    }
    // Serialize message field [pose]
    obj.pose.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [duration] has the right length
    if (obj.duration.length !== 4) {
      throw new Error('Unable to serialize array field duration - length must be 4')
    }
    // Serialize message field [duration]
    bufferOffset = _arraySerializer.float64(obj.duration, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TaskCommand
    let len;
    let data = new TaskCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [end_effector]
    data.end_effector = _arrayDeserializer.bool(buffer, bufferOffset, 4)
    // Deserialize message field [mode]
    data.mode = _arrayDeserializer.uint32(buffer, bufferOffset, 4)
    // Deserialize message field [pose]
    len = 4;
    data.pose = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pose[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [duration]
    data.duration = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 276;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dyros_jet_msgs/TaskCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '636def6d1efe8a9dd71ed012354704b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    # end_effector = true -> use 
    bool[4] end_effector
    uint32 EE_LEFT_FOOT = 0
    uint32 EE_RIGHT_FOOT = 1
    uint32 EE_LEFT_HAND = 2
    uint32 EE_RIGHT_HAND = 3
    
    uint32[4] mode
    uint32 RELATIVE=0
    uint32 ABSOLUTE=1
    
    geometry_msgs/Pose[4] pose
    
    float64[4] duration
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TaskCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.end_effector !== undefined) {
      resolved.end_effector = msg.end_effector;
    }
    else {
      resolved.end_effector = new Array(4).fill(0)
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = new Array(4).fill(0)
    }

    if (msg.pose !== undefined) {
      resolved.pose = new Array(4)
      for (let i = 0; i < resolved.pose.length; ++i) {
        if (msg.pose.length > i) {
          resolved.pose[i] = geometry_msgs.msg.Pose.Resolve(msg.pose[i]);
        }
        else {
          resolved.pose[i] = new geometry_msgs.msg.Pose();
        }
      }
    }
    else {
      resolved.pose = new Array(4).fill(new geometry_msgs.msg.Pose())
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = new Array(4).fill(0)
    }

    return resolved;
    }
};

// Constants for message
TaskCommand.Constants = {
  EE_LEFT_FOOT: 0,
  EE_RIGHT_FOOT: 1,
  EE_LEFT_HAND: 2,
  EE_RIGHT_HAND: 3,
  RELATIVE: 0,
  ABSOLUTE: 1,
}

module.exports = TaskCommand;
