; Auto-generated. Do not edit!


(cl:in-package dyros_jet_msgs-msg)


;//! \htmlinclude TaskCommand.msg.html

(cl:defclass <TaskCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (end_effector
    :reader end_effector
    :initarg :end_effector
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 4 :element-type 'cl:boolean :initial-element cl:nil))
   (mode
    :reader mode
    :initarg :mode
    :type (cl:vector cl:integer)
   :initform (cl:make-array 4 :element-type 'cl:integer :initial-element 0))
   (pose
    :reader pose
    :initarg :pose
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 4 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (duration
    :reader duration
    :initarg :duration
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass TaskCommand (<TaskCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TaskCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TaskCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dyros_jet_msgs-msg:<TaskCommand> is deprecated: use dyros_jet_msgs-msg:TaskCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TaskCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:header-val is deprecated.  Use dyros_jet_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'end_effector-val :lambda-list '(m))
(cl:defmethod end_effector-val ((m <TaskCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:end_effector-val is deprecated.  Use dyros_jet_msgs-msg:end_effector instead.")
  (end_effector m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <TaskCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:mode-val is deprecated.  Use dyros_jet_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <TaskCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:pose-val is deprecated.  Use dyros_jet_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <TaskCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:duration-val is deprecated.  Use dyros_jet_msgs-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TaskCommand>)))
    "Constants for message type '<TaskCommand>"
  '((:EE_LEFT_FOOT . 0)
    (:EE_RIGHT_FOOT . 1)
    (:EE_LEFT_HAND . 2)
    (:EE_RIGHT_HAND . 3)
    (:RELATIVE . 0)
    (:ABSOLUTE . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TaskCommand)))
    "Constants for message type 'TaskCommand"
  '((:EE_LEFT_FOOT . 0)
    (:EE_RIGHT_FOOT . 1)
    (:EE_LEFT_HAND . 2)
    (:EE_RIGHT_HAND . 3)
    (:RELATIVE . 0)
    (:ABSOLUTE . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TaskCommand>) ostream)
  "Serializes a message object of type '<TaskCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'end_effector))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'mode))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pose))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'duration))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TaskCommand>) istream)
  "Deserializes a message object of type '<TaskCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'end_effector) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'end_effector)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  (cl:setf (cl:slot-value msg 'mode) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'mode)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'pose) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'pose)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'duration) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'duration)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TaskCommand>)))
  "Returns string type for a message object of type '<TaskCommand>"
  "dyros_jet_msgs/TaskCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TaskCommand)))
  "Returns string type for a message object of type 'TaskCommand"
  "dyros_jet_msgs/TaskCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TaskCommand>)))
  "Returns md5sum for a message object of type '<TaskCommand>"
  "636def6d1efe8a9dd71ed012354704b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TaskCommand)))
  "Returns md5sum for a message object of type 'TaskCommand"
  "636def6d1efe8a9dd71ed012354704b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TaskCommand>)))
  "Returns full string definition for message of type '<TaskCommand>"
  (cl:format cl:nil "std_msgs/Header header~%~%# end_effector = true -> use ~%bool[4] end_effector~%uint32 EE_LEFT_FOOT = 0~%uint32 EE_RIGHT_FOOT = 1~%uint32 EE_LEFT_HAND = 2~%uint32 EE_RIGHT_HAND = 3~%~%uint32[4] mode~%uint32 RELATIVE=0~%uint32 ABSOLUTE=1~%~%geometry_msgs/Pose[4] pose~%~%float64[4] duration~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TaskCommand)))
  "Returns full string definition for message of type 'TaskCommand"
  (cl:format cl:nil "std_msgs/Header header~%~%# end_effector = true -> use ~%bool[4] end_effector~%uint32 EE_LEFT_FOOT = 0~%uint32 EE_RIGHT_FOOT = 1~%uint32 EE_LEFT_HAND = 2~%uint32 EE_RIGHT_HAND = 3~%~%uint32[4] mode~%uint32 RELATIVE=0~%uint32 ABSOLUTE=1~%~%geometry_msgs/Pose[4] pose~%~%float64[4] duration~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TaskCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'end_effector) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mode) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'duration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TaskCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'TaskCommand
    (cl:cons ':header (header msg))
    (cl:cons ':end_effector (end_effector msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':duration (duration msg))
))
