// Auto-generated. Do not edit!

// (in-package motioncontroller.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class status {
  constructor() {
    this.errorcode = '';
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type status
    // Serialize message field [errorcode]
    bufferInfo = _serializer.string(obj.errorcode, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type status
    let tmp;
    let len;
    let data = new status();
    // Deserialize message field [errorcode]
    tmp = _deserializer.string(buffer);
    data.errorcode = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'motioncontroller/status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec7d276cbbe91ad63105ca2d4c757803';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string errorcode
    
    `;
  }

};

module.exports = status;
