// Auto-generated. Do not edit!

// (in-package dyros_jet_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class WalkingCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.walk_mode = null;
      this.compensator_mode = null;
      this.ik_mode = null;
      this.heel_toe = null;
      this.first_foot_step = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.height = null;
      this.theta = null;
      this.step_length_x = null;
      this.step_length_y = null;
      this.walking_pattern = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('walk_mode')) {
        this.walk_mode = initObj.walk_mode
      }
      else {
        this.walk_mode = 0;
      }
      if (initObj.hasOwnProperty('compensator_mode')) {
        this.compensator_mode = initObj.compensator_mode
      }
      else {
        this.compensator_mode = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('ik_mode')) {
        this.ik_mode = initObj.ik_mode
      }
      else {
        this.ik_mode = 0;
      }
      if (initObj.hasOwnProperty('heel_toe')) {
        this.heel_toe = initObj.heel_toe
      }
      else {
        this.heel_toe = false;
      }
      if (initObj.hasOwnProperty('first_foot_step')) {
        this.first_foot_step = initObj.first_foot_step
      }
      else {
        this.first_foot_step = false;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('theta')) {
        this.theta = initObj.theta
      }
      else {
        this.theta = 0.0;
      }
      if (initObj.hasOwnProperty('step_length_x')) {
        this.step_length_x = initObj.step_length_x
      }
      else {
        this.step_length_x = 0.0;
      }
      if (initObj.hasOwnProperty('step_length_y')) {
        this.step_length_y = initObj.step_length_y
      }
      else {
        this.step_length_y = 0.0;
      }
      if (initObj.hasOwnProperty('walking_pattern')) {
        this.walking_pattern = initObj.walking_pattern
      }
      else {
        this.walking_pattern = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WalkingCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [walk_mode]
    bufferOffset = _serializer.uint32(obj.walk_mode, buffer, bufferOffset);
    // Check that the constant length array field [compensator_mode] has the right length
    if (obj.compensator_mode.length !== 2) {
      throw new Error('Unable to serialize array field compensator_mode - length must be 2')
    }
    // Serialize message field [compensator_mode]
    bufferOffset = _arraySerializer.bool(obj.compensator_mode, buffer, bufferOffset, 2);
    // Serialize message field [ik_mode]
    bufferOffset = _serializer.uint32(obj.ik_mode, buffer, bufferOffset);
    // Serialize message field [heel_toe]
    bufferOffset = _serializer.bool(obj.heel_toe, buffer, bufferOffset);
    // Serialize message field [first_foot_step]
    bufferOffset = _serializer.bool(obj.first_foot_step, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.float32(obj.height, buffer, bufferOffset);
    // Serialize message field [theta]
    bufferOffset = _serializer.float32(obj.theta, buffer, bufferOffset);
    // Serialize message field [step_length_x]
    bufferOffset = _serializer.float32(obj.step_length_x, buffer, bufferOffset);
    // Serialize message field [step_length_y]
    bufferOffset = _serializer.float32(obj.step_length_y, buffer, bufferOffset);
    // Serialize message field [walking_pattern]
    bufferOffset = _serializer.bool(obj.walking_pattern, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WalkingCommand
    let len;
    let data = new WalkingCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [walk_mode]
    data.walk_mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [compensator_mode]
    data.compensator_mode = _arrayDeserializer.bool(buffer, bufferOffset, 2)
    // Deserialize message field [ik_mode]
    data.ik_mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [heel_toe]
    data.heel_toe = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [first_foot_step]
    data.first_foot_step = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta]
    data.theta = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [step_length_x]
    data.step_length_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [step_length_y]
    data.step_length_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [walking_pattern]
    data.walking_pattern = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 41;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dyros_jet_msgs/WalkingCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c98f46b805c25b4897e6489a2543122';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint32 walk_mode
    uint32 NONE = 0
    uint32 STATIC_WALKING = 1
    
    bool[2] compensator_mode
    uint32 HIP_COMPENSTOR = 0
    uint32 EXTERNAL_ENCODER = 1
    
    uint32 ik_mode
    uint32 IK=0
    uint32 JACOBIAN = 1
    uint32 QP = 2
    
    bool heel_toe
    
    bool first_foot_step
    
    float32 x
    float32 y
    float32 z
    float32 height
    float32 theta
    float32 step_length_x
    float32 step_length_y
    
    bool walking_pattern
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WalkingCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.walk_mode !== undefined) {
      resolved.walk_mode = msg.walk_mode;
    }
    else {
      resolved.walk_mode = 0
    }

    if (msg.compensator_mode !== undefined) {
      resolved.compensator_mode = msg.compensator_mode;
    }
    else {
      resolved.compensator_mode = new Array(2).fill(0)
    }

    if (msg.ik_mode !== undefined) {
      resolved.ik_mode = msg.ik_mode;
    }
    else {
      resolved.ik_mode = 0
    }

    if (msg.heel_toe !== undefined) {
      resolved.heel_toe = msg.heel_toe;
    }
    else {
      resolved.heel_toe = false
    }

    if (msg.first_foot_step !== undefined) {
      resolved.first_foot_step = msg.first_foot_step;
    }
    else {
      resolved.first_foot_step = false
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.theta !== undefined) {
      resolved.theta = msg.theta;
    }
    else {
      resolved.theta = 0.0
    }

    if (msg.step_length_x !== undefined) {
      resolved.step_length_x = msg.step_length_x;
    }
    else {
      resolved.step_length_x = 0.0
    }

    if (msg.step_length_y !== undefined) {
      resolved.step_length_y = msg.step_length_y;
    }
    else {
      resolved.step_length_y = 0.0
    }

    if (msg.walking_pattern !== undefined) {
      resolved.walking_pattern = msg.walking_pattern;
    }
    else {
      resolved.walking_pattern = false
    }

    return resolved;
    }
};

// Constants for message
WalkingCommand.Constants = {
  NONE: 0,
  STATIC_WALKING: 1,
  HIP_COMPENSTOR: 0,
  EXTERNAL_ENCODER: 1,
  IK: 0,
  JACOBIAN: 1,
  QP: 2,
}

module.exports = WalkingCommand;
