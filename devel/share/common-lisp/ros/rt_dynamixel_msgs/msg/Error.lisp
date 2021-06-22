; Auto-generated. Do not edit!


(cl:in-package rt_dynamixel_msgs-msg)


;//! \htmlinclude Error.msg.html

(cl:defclass <Error> (roslisp-msg-protocol:ros-message)
  ((error_code
    :reader error_code
    :initarg :error_code
    :type cl:integer
    :initform 0)
   (error_detail
    :reader error_detail
    :initarg :error_detail
    :type cl:string
    :initform ""))
)

(cl:defclass Error (<Error>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Error>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Error)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rt_dynamixel_msgs-msg:<Error> is deprecated: use rt_dynamixel_msgs-msg:Error instead.")))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <Error>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_dynamixel_msgs-msg:error_code-val is deprecated.  Use rt_dynamixel_msgs-msg:error_code instead.")
  (error_code m))

(cl:ensure-generic-function 'error_detail-val :lambda-list '(m))
(cl:defmethod error_detail-val ((m <Error>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_dynamixel_msgs-msg:error_detail-val is deprecated.  Use rt_dynamixel_msgs-msg:error_detail instead.")
  (error_detail m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Error>)))
    "Constants for message type '<Error>"
  '((:SER_INIT_FAILED . 1)
    (:OFFLINE_DETECT . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Error)))
    "Constants for message type 'Error"
  '((:SER_INIT_FAILED . 1)
    (:OFFLINE_DETECT . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Error>) ostream)
  "Serializes a message object of type '<Error>"
  (cl:let* ((signed (cl:slot-value msg 'error_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'error_detail))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'error_detail))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Error>) istream)
  "Deserializes a message object of type '<Error>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_detail) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'error_detail) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Error>)))
  "Returns string type for a message object of type '<Error>"
  "rt_dynamixel_msgs/Error")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Error)))
  "Returns string type for a message object of type 'Error"
  "rt_dynamixel_msgs/Error")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Error>)))
  "Returns md5sum for a message object of type '<Error>"
  "d97d350edf68e4d52368560032b073fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Error)))
  "Returns md5sum for a message object of type 'Error"
  "d97d350edf68e4d52368560032b073fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Error>)))
  "Returns full string definition for message of type '<Error>"
  (cl:format cl:nil "~%int32 SER_INIT_FAILED=1~%int32 OFFLINE_DETECT=2~%~%int32 error_code~%string error_detail~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Error)))
  "Returns full string definition for message of type 'Error"
  (cl:format cl:nil "~%int32 SER_INIT_FAILED=1~%int32 OFFLINE_DETECT=2~%~%int32 error_code~%string error_detail~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Error>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'error_detail))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Error>))
  "Converts a ROS message object to a list"
  (cl:list 'Error
    (cl:cons ':error_code (error_code msg))
    (cl:cons ':error_detail (error_detail msg))
))
