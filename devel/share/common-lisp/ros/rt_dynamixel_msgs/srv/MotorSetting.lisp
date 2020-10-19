; Auto-generated. Do not edit!


(cl:in-package rt_dynamixel_msgs-srv)


;//! \htmlinclude MotorSetting-request.msg.html

(cl:defclass <MotorSetting-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0)
   (fvalue
    :reader fvalue
    :initarg :fvalue
    :type cl:float
    :initform 0.0))
)

(cl:defclass MotorSetting-request (<MotorSetting-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorSetting-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorSetting-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rt_dynamixel_msgs-srv:<MotorSetting-request> is deprecated: use rt_dynamixel_msgs-srv:MotorSetting-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <MotorSetting-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_dynamixel_msgs-srv:mode-val is deprecated.  Use rt_dynamixel_msgs-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <MotorSetting-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_dynamixel_msgs-srv:id-val is deprecated.  Use rt_dynamixel_msgs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <MotorSetting-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_dynamixel_msgs-srv:value-val is deprecated.  Use rt_dynamixel_msgs-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'fvalue-val :lambda-list '(m))
(cl:defmethod fvalue-val ((m <MotorSetting-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_dynamixel_msgs-srv:fvalue-val is deprecated.  Use rt_dynamixel_msgs-srv:fvalue instead.")
  (fvalue m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<MotorSetting-request>)))
    "Constants for message type '<MotorSetting-request>"
  '((:SET_HOMING_OFFSET . 17)
    (:GET_HOMING_OFFSET . 18)
    (:SET_TORQUE_ENABLE . 19)
    (:SET_GOAL_POSITION . 20))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'MotorSetting-request)))
    "Constants for message type 'MotorSetting-request"
  '((:SET_HOMING_OFFSET . 17)
    (:GET_HOMING_OFFSET . 18)
    (:SET_TORQUE_ENABLE . 19)
    (:SET_GOAL_POSITION . 20))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorSetting-request>) ostream)
  "Serializes a message object of type '<MotorSetting-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'fvalue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorSetting-request>) istream)
  "Deserializes a message object of type '<MotorSetting-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fvalue) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorSetting-request>)))
  "Returns string type for a service object of type '<MotorSetting-request>"
  "rt_dynamixel_msgs/MotorSettingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorSetting-request)))
  "Returns string type for a service object of type 'MotorSetting-request"
  "rt_dynamixel_msgs/MotorSettingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorSetting-request>)))
  "Returns md5sum for a message object of type '<MotorSetting-request>"
  "1c80fb3348bbb124fc72557f61e2f00b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorSetting-request)))
  "Returns md5sum for a message object of type 'MotorSetting-request"
  "1c80fb3348bbb124fc72557f61e2f00b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorSetting-request>)))
  "Returns full string definition for message of type '<MotorSetting-request>"
  (cl:format cl:nil "~%uint8 SET_HOMING_OFFSET=17~%uint8 GET_HOMING_OFFSET=18~%uint8 SET_TORQUE_ENABLE=19~%uint8 SET_GOAL_POSITION=20~%~%int32 mode~%int32 id~%int64 value~%float64 fvalue~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorSetting-request)))
  "Returns full string definition for message of type 'MotorSetting-request"
  (cl:format cl:nil "~%uint8 SET_HOMING_OFFSET=17~%uint8 GET_HOMING_OFFSET=18~%uint8 SET_TORQUE_ENABLE=19~%uint8 SET_GOAL_POSITION=20~%~%int32 mode~%int32 id~%int64 value~%float64 fvalue~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorSetting-request>))
  (cl:+ 0
     4
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorSetting-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorSetting-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':id (id msg))
    (cl:cons ':value (value msg))
    (cl:cons ':fvalue (fvalue msg))
))
;//! \htmlinclude MotorSetting-response.msg.html

(cl:defclass <MotorSetting-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass MotorSetting-response (<MotorSetting-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorSetting-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorSetting-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rt_dynamixel_msgs-srv:<MotorSetting-response> is deprecated: use rt_dynamixel_msgs-srv:MotorSetting-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <MotorSetting-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_dynamixel_msgs-srv:result-val is deprecated.  Use rt_dynamixel_msgs-srv:result instead.")
  (result m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <MotorSetting-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_dynamixel_msgs-srv:value-val is deprecated.  Use rt_dynamixel_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorSetting-response>) ostream)
  "Serializes a message object of type '<MotorSetting-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorSetting-response>) istream)
  "Deserializes a message object of type '<MotorSetting-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorSetting-response>)))
  "Returns string type for a service object of type '<MotorSetting-response>"
  "rt_dynamixel_msgs/MotorSettingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorSetting-response)))
  "Returns string type for a service object of type 'MotorSetting-response"
  "rt_dynamixel_msgs/MotorSettingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorSetting-response>)))
  "Returns md5sum for a message object of type '<MotorSetting-response>"
  "1c80fb3348bbb124fc72557f61e2f00b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorSetting-response)))
  "Returns md5sum for a message object of type 'MotorSetting-response"
  "1c80fb3348bbb124fc72557f61e2f00b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorSetting-response>)))
  "Returns full string definition for message of type '<MotorSetting-response>"
  (cl:format cl:nil "int32 result~%int64 value~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorSetting-response)))
  "Returns full string definition for message of type 'MotorSetting-response"
  (cl:format cl:nil "int32 result~%int64 value~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorSetting-response>))
  (cl:+ 0
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorSetting-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorSetting-response
    (cl:cons ':result (result msg))
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MotorSetting)))
  'MotorSetting-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MotorSetting)))
  'MotorSetting-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorSetting)))
  "Returns string type for a service object of type '<MotorSetting>"
  "rt_dynamixel_msgs/MotorSetting")