// Auto-generated. Do not edit!

// (in-package rt_dynamixel_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class MotorSettingRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.id = null;
      this.value = null;
      this.fvalue = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
      if (initObj.hasOwnProperty('fvalue')) {
        this.fvalue = initObj.fvalue
      }
      else {
        this.fvalue = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorSettingRequest
    // Serialize message field [mode]
    bufferOffset = _serializer.int32(obj.mode, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.int64(obj.value, buffer, bufferOffset);
    // Serialize message field [fvalue]
    bufferOffset = _serializer.float64(obj.fvalue, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorSettingRequest
    let len;
    let data = new MotorSettingRequest(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [fvalue]
    data.fvalue = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rt_dynamixel_msgs/MotorSettingRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa57006293ff4f56089dba8a7218e23b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint8 SET_HOMING_OFFSET=17
    uint8 GET_HOMING_OFFSET=18
    uint8 SET_TORQUE_ENABLE=19
    uint8 SET_GOAL_POSITION=20
    
    int32 mode
    int32 id
    int64 value
    float64 fvalue
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorSettingRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    if (msg.fvalue !== undefined) {
      resolved.fvalue = msg.fvalue;
    }
    else {
      resolved.fvalue = 0.0
    }

    return resolved;
    }
};

// Constants for message
MotorSettingRequest.Constants = {
  SET_HOMING_OFFSET: 17,
  GET_HOMING_OFFSET: 18,
  SET_TORQUE_ENABLE: 19,
  SET_GOAL_POSITION: 20,
}

class MotorSettingResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorSettingResponse
    // Serialize message field [result]
    bufferOffset = _serializer.int32(obj.result, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.int64(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorSettingResponse
    let len;
    let data = new MotorSettingResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rt_dynamixel_msgs/MotorSettingResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d24c7dd53dc7c2fd1b5a1f364f18349';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 result
    int64 value
    
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorSettingResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: MotorSettingRequest,
  Response: MotorSettingResponse,
  md5sum() { return '1c80fb3348bbb124fc72557f61e2f00b'; },
  datatype() { return 'rt_dynamixel_msgs/MotorSetting'; }
};
