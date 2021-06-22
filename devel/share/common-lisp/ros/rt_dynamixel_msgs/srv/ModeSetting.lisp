; Auto-generated. Do not edit!


(cl:in-package rt_dynamixel_msgs-srv)


;//! \htmlinclude ModeSetting-request.msg.html

(cl:defclass <ModeSetting-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass ModeSetting-request (<ModeSetting-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModeSetting-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModeSetting-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rt_dynamixel_msgs-srv:<ModeSetting-request> is deprecated: use rt_dynamixel_msgs-srv:ModeSetting-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <ModeSetting-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_dynamixel_msgs-srv:mode-val is deprecated.  Use rt_dynamixel_msgs-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ModeSetting-request>)))
    "Constants for message type '<ModeSetting-request>"
  '((:DISABLE . 0)
    (:CONTROL_RUN . 1)
    (:SETTING . 2)
    (:TEST . 500))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ModeSetting-request)))
    "Constants for message type 'ModeSetting-request"
  '((:DISABLE . 0)
    (:CONTROL_RUN . 1)
    (:SETTING . 2)
    (:TEST . 500))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModeSetting-request>) ostream)
  "Serializes a message object of type '<ModeSetting-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModeSetting-request>) istream)
  "Deserializes a message object of type '<ModeSetting-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModeSetting-request>)))
  "Returns string type for a service object of type '<ModeSetting-request>"
  "rt_dynamixel_msgs/ModeSettingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeSetting-request)))
  "Returns string type for a service object of type 'ModeSetting-request"
  "rt_dynamixel_msgs/ModeSettingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModeSetting-request>)))
  "Returns md5sum for a message object of type '<ModeSetting-request>"
  "84eab6ad37979cc9c127ed5e5d0e8fdb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModeSetting-request)))
  "Returns md5sum for a message object of type 'ModeSetting-request"
  "84eab6ad37979cc9c127ed5e5d0e8fdb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModeSetting-request>)))
  "Returns full string definition for message of type '<ModeSetting-request>"
  (cl:format cl:nil "~%int32 DISABLE=0~%int32 CONTROL_RUN=1~%int32 SETTING=2~%int32 TEST=500~%~%int32 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModeSetting-request)))
  "Returns full string definition for message of type 'ModeSetting-request"
  (cl:format cl:nil "~%int32 DISABLE=0~%int32 CONTROL_RUN=1~%int32 SETTING=2~%int32 TEST=500~%~%int32 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModeSetting-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModeSetting-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ModeSetting-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude ModeSetting-response.msg.html

(cl:defclass <ModeSetting-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass ModeSetting-response (<ModeSetting-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModeSetting-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModeSetting-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rt_dynamixel_msgs-srv:<ModeSetting-response> is deprecated: use rt_dynamixel_msgs-srv:ModeSetting-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <ModeSetting-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_dynamixel_msgs-srv:result-val is deprecated.  Use rt_dynamixel_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModeSetting-response>) ostream)
  "Serializes a message object of type '<ModeSetting-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModeSetting-response>) istream)
  "Deserializes a message object of type '<ModeSetting-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModeSetting-response>)))
  "Returns string type for a service object of type '<ModeSetting-response>"
  "rt_dynamixel_msgs/ModeSettingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeSetting-response)))
  "Returns string type for a service object of type 'ModeSetting-response"
  "rt_dynamixel_msgs/ModeSettingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModeSetting-response>)))
  "Returns md5sum for a message object of type '<ModeSetting-response>"
  "84eab6ad37979cc9c127ed5e5d0e8fdb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModeSetting-response)))
  "Returns md5sum for a message object of type 'ModeSetting-response"
  "84eab6ad37979cc9c127ed5e5d0e8fdb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModeSetting-response>)))
  "Returns full string definition for message of type '<ModeSetting-response>"
  (cl:format cl:nil "int32 result~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModeSetting-response)))
  "Returns full string definition for message of type 'ModeSetting-response"
  (cl:format cl:nil "int32 result~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModeSetting-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModeSetting-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ModeSetting-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ModeSetting)))
  'ModeSetting-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ModeSetting)))
  'ModeSetting-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeSetting)))
  "Returns string type for a service object of type '<ModeSetting>"
  "rt_dynamixel_msgs/ModeSetting")