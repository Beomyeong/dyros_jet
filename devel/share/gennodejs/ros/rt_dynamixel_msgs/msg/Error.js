// Auto-generated. Do not edit!

// (in-package rt_dynamixel_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Error {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.error_code = null;
      this.error_detail = null;
    }
    else {
      if (initObj.hasOwnProperty('error_code')) {
        this.error_code = initObj.error_code
      }
      else {
        this.error_code = 0;
      }
      if (initObj.hasOwnProperty('error_detail')) {
        this.error_detail = initObj.error_detail
      }
      else {
        this.error_detail = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Error
    // Serialize message field [error_code]
    bufferOffset = _serializer.int32(obj.error_code, buffer, bufferOffset);
    // Serialize message field [error_detail]
    bufferOffset = _serializer.string(obj.error_detail, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Error
    let len;
    let data = new Error(null);
    // Deserialize message field [error_code]
    data.error_code = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [error_detail]
    data.error_detail = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.error_detail.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rt_dynamixel_msgs/Error';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd97d350edf68e4d52368560032b073fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int32 SER_INIT_FAILED=1
    int32 OFFLINE_DETECT=2
    
    int32 error_code
    string error_detail
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Error(null);
    if (msg.error_code !== undefined) {
      resolved.error_code = msg.error_code;
    }
    else {
      resolved.error_code = 0
    }

    if (msg.error_detail !== undefined) {
      resolved.error_detail = msg.error_detail;
    }
    else {
      resolved.error_detail = ''
    }

    return resolved;
    }
};

// Constants for message
Error.Constants = {
  SER_INIT_FAILED: 1,
  OFFLINE_DETECT: 2,
}

module.exports = Error;
