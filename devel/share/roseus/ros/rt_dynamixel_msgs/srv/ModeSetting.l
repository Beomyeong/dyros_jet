;; Auto-generated. Do not edit!


(when (boundp 'rt_dynamixel_msgs::ModeSetting)
  (if (not (find-package "RT_DYNAMIXEL_MSGS"))
    (make-package "RT_DYNAMIXEL_MSGS"))
  (shadow 'ModeSetting (find-package "RT_DYNAMIXEL_MSGS")))
(unless (find-package "RT_DYNAMIXEL_MSGS::MODESETTING")
  (make-package "RT_DYNAMIXEL_MSGS::MODESETTING"))
(unless (find-package "RT_DYNAMIXEL_MSGS::MODESETTINGREQUEST")
  (make-package "RT_DYNAMIXEL_MSGS::MODESETTINGREQUEST"))
(unless (find-package "RT_DYNAMIXEL_MSGS::MODESETTINGRESPONSE")
  (make-package "RT_DYNAMIXEL_MSGS::MODESETTINGRESPONSE"))

(in-package "ROS")





(intern "*DISABLE*" (find-package "RT_DYNAMIXEL_MSGS::MODESETTINGREQUEST"))
(shadow '*DISABLE* (find-package "RT_DYNAMIXEL_MSGS::MODESETTINGREQUEST"))
(defconstant rt_dynamixel_msgs::ModeSettingRequest::*DISABLE* 0)
(intern "*CONTROL_RUN*" (find-package "RT_DYNAMIXEL_MSGS::MODESETTINGREQUEST"))
(shadow '*CONTROL_RUN* (find-package "RT_DYNAMIXEL_MSGS::MODESETTINGREQUEST"))
(defconstant rt_dynamixel_msgs::ModeSettingRequest::*CONTROL_RUN* 1)
(intern "*SETTING*" (find-package "RT_DYNAMIXEL_MSGS::MODESETTINGREQUEST"))
(shadow '*SETTING* (find-package "RT_DYNAMIXEL_MSGS::MODESETTINGREQUEST"))
(defconstant rt_dynamixel_msgs::ModeSettingRequest::*SETTING* 2)
(intern "*TEST*" (find-package "RT_DYNAMIXEL_MSGS::MODESETTINGREQUEST"))
(shadow '*TEST* (find-package "RT_DYNAMIXEL_MSGS::MODESETTINGREQUEST"))
(defconstant rt_dynamixel_msgs::ModeSettingRequest::*TEST* 500)
(defclass rt_dynamixel_msgs::ModeSettingRequest
  :super ros::object
  :slots (_mode ))

(defmethod rt_dynamixel_msgs::ModeSettingRequest
  (:init
   (&key
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; int32 _mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _mode
       (write-long _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass rt_dynamixel_msgs::ModeSettingResponse
  :super ros::object
  :slots (_result ))

(defmethod rt_dynamixel_msgs::ModeSettingResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass rt_dynamixel_msgs::ModeSetting
  :super ros::object
  :slots ())

(setf (get rt_dynamixel_msgs::ModeSetting :md5sum-) "84eab6ad37979cc9c127ed5e5d0e8fdb")
(setf (get rt_dynamixel_msgs::ModeSetting :datatype-) "rt_dynamixel_msgs/ModeSetting")
(setf (get rt_dynamixel_msgs::ModeSetting :request) rt_dynamixel_msgs::ModeSettingRequest)
(setf (get rt_dynamixel_msgs::ModeSetting :response) rt_dynamixel_msgs::ModeSettingResponse)

(defmethod rt_dynamixel_msgs::ModeSettingRequest
  (:response () (instance rt_dynamixel_msgs::ModeSettingResponse :init)))

(setf (get rt_dynamixel_msgs::ModeSettingRequest :md5sum-) "84eab6ad37979cc9c127ed5e5d0e8fdb")
(setf (get rt_dynamixel_msgs::ModeSettingRequest :datatype-) "rt_dynamixel_msgs/ModeSettingRequest")
(setf (get rt_dynamixel_msgs::ModeSettingRequest :definition-)
      "
int32 DISABLE=0
int32 CONTROL_RUN=1
int32 SETTING=2
int32 TEST=500

int32 mode
---
int32 result






")

(setf (get rt_dynamixel_msgs::ModeSettingResponse :md5sum-) "84eab6ad37979cc9c127ed5e5d0e8fdb")
(setf (get rt_dynamixel_msgs::ModeSettingResponse :datatype-) "rt_dynamixel_msgs/ModeSettingResponse")
(setf (get rt_dynamixel_msgs::ModeSettingResponse :definition-)
      "
int32 DISABLE=0
int32 CONTROL_RUN=1
int32 SETTING=2
int32 TEST=500

int32 mode
---
int32 result






")



(provide :rt_dynamixel_msgs/ModeSetting "84eab6ad37979cc9c127ed5e5d0e8fdb")


