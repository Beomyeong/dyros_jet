;; Auto-generated. Do not edit!


(when (boundp 'rt_dynamixel_msgs::MotorSetting)
  (if (not (find-package "RT_DYNAMIXEL_MSGS"))
    (make-package "RT_DYNAMIXEL_MSGS"))
  (shadow 'MotorSetting (find-package "RT_DYNAMIXEL_MSGS")))
(unless (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTING")
  (make-package "RT_DYNAMIXEL_MSGS::MOTORSETTING"))
(unless (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGREQUEST")
  (make-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGREQUEST"))
(unless (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGRESPONSE")
  (make-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGRESPONSE"))

(in-package "ROS")





(intern "*SET_HOMING_OFFSET*" (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGREQUEST"))
(shadow '*SET_HOMING_OFFSET* (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGREQUEST"))
(defconstant rt_dynamixel_msgs::MotorSettingRequest::*SET_HOMING_OFFSET* 17)
(intern "*GET_HOMING_OFFSET*" (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGREQUEST"))
(shadow '*GET_HOMING_OFFSET* (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGREQUEST"))
(defconstant rt_dynamixel_msgs::MotorSettingRequest::*GET_HOMING_OFFSET* 18)
(intern "*SET_TORQUE_ENABLE*" (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGREQUEST"))
(shadow '*SET_TORQUE_ENABLE* (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGREQUEST"))
(defconstant rt_dynamixel_msgs::MotorSettingRequest::*SET_TORQUE_ENABLE* 19)
(intern "*SET_GOAL_POSITION*" (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGREQUEST"))
(shadow '*SET_GOAL_POSITION* (find-package "RT_DYNAMIXEL_MSGS::MOTORSETTINGREQUEST"))
(defconstant rt_dynamixel_msgs::MotorSettingRequest::*SET_GOAL_POSITION* 20)
(defclass rt_dynamixel_msgs::MotorSettingRequest
  :super ros::object
  :slots (_mode _id _value _fvalue ))

(defmethod rt_dynamixel_msgs::MotorSettingRequest
  (:init
   (&key
    ((:mode __mode) 0)
    ((:id __id) 0)
    ((:value __value) 0)
    ((:fvalue __fvalue) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _id (round __id))
   (setq _value (round __value))
   (setq _fvalue (float __fvalue))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:fvalue
   (&optional __fvalue)
   (if __fvalue (setq _fvalue __fvalue)) _fvalue)
  (:serialization-length
   ()
   (+
    ;; int32 _mode
    4
    ;; int32 _id
    4
    ;; int64 _value
    8
    ;; float64 _fvalue
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _mode
       (write-long _mode s)
     ;; int32 _id
       (write-long _id s)
     ;; int64 _value
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _value (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _value) (= (length (_value . bv)) 2)) ;; bignum
              (write-long (ash (elt (_value . bv) 0) 0) s)
              (write-long (ash (elt (_value . bv) 1) -1) s))
             ((and (class _value) (= (length (_value . bv)) 1)) ;; big1
              (write-long (elt (_value . bv) 0) s)
              (write-long (if (>= _value 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _value s)(write-long (if (>= _value 0) 0 #xffffffff) s)))
     ;; float64 _fvalue
       (sys::poke _fvalue (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _value
#+(or :alpha :irix6 :x86_64)
      (setf _value (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _value (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _fvalue
     (setq _fvalue (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass rt_dynamixel_msgs::MotorSettingResponse
  :super ros::object
  :slots (_result _value ))

(defmethod rt_dynamixel_msgs::MotorSettingResponse
  (:init
   (&key
    ((:result __result) 0)
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   (setq _value (round __value))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ;; int64 _value
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;; int64 _value
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _value (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _value) (= (length (_value . bv)) 2)) ;; bignum
              (write-long (ash (elt (_value . bv) 0) 0) s)
              (write-long (ash (elt (_value . bv) 1) -1) s))
             ((and (class _value) (= (length (_value . bv)) 1)) ;; big1
              (write-long (elt (_value . bv) 0) s)
              (write-long (if (>= _value 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _value s)(write-long (if (>= _value 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _value
#+(or :alpha :irix6 :x86_64)
      (setf _value (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _value (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass rt_dynamixel_msgs::MotorSetting
  :super ros::object
  :slots ())

(setf (get rt_dynamixel_msgs::MotorSetting :md5sum-) "1c80fb3348bbb124fc72557f61e2f00b")
(setf (get rt_dynamixel_msgs::MotorSetting :datatype-) "rt_dynamixel_msgs/MotorSetting")
(setf (get rt_dynamixel_msgs::MotorSetting :request) rt_dynamixel_msgs::MotorSettingRequest)
(setf (get rt_dynamixel_msgs::MotorSetting :response) rt_dynamixel_msgs::MotorSettingResponse)

(defmethod rt_dynamixel_msgs::MotorSettingRequest
  (:response () (instance rt_dynamixel_msgs::MotorSettingResponse :init)))

(setf (get rt_dynamixel_msgs::MotorSettingRequest :md5sum-) "1c80fb3348bbb124fc72557f61e2f00b")
(setf (get rt_dynamixel_msgs::MotorSettingRequest :datatype-) "rt_dynamixel_msgs/MotorSettingRequest")
(setf (get rt_dynamixel_msgs::MotorSettingRequest :definition-)
      "
uint8 SET_HOMING_OFFSET=17
uint8 GET_HOMING_OFFSET=18
uint8 SET_TORQUE_ENABLE=19
uint8 SET_GOAL_POSITION=20

int32 mode
int32 id
int64 value
float64 fvalue
---
int32 result
int64 value




")

(setf (get rt_dynamixel_msgs::MotorSettingResponse :md5sum-) "1c80fb3348bbb124fc72557f61e2f00b")
(setf (get rt_dynamixel_msgs::MotorSettingResponse :datatype-) "rt_dynamixel_msgs/MotorSettingResponse")
(setf (get rt_dynamixel_msgs::MotorSettingResponse :definition-)
      "
uint8 SET_HOMING_OFFSET=17
uint8 GET_HOMING_OFFSET=18
uint8 SET_TORQUE_ENABLE=19
uint8 SET_GOAL_POSITION=20

int32 mode
int32 id
int64 value
float64 fvalue
---
int32 result
int64 value




")



(provide :rt_dynamixel_msgs/MotorSetting "1c80fb3348bbb124fc72557f61e2f00b")


