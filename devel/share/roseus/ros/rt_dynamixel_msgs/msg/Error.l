;; Auto-generated. Do not edit!


(when (boundp 'rt_dynamixel_msgs::Error)
  (if (not (find-package "RT_DYNAMIXEL_MSGS"))
    (make-package "RT_DYNAMIXEL_MSGS"))
  (shadow 'Error (find-package "RT_DYNAMIXEL_MSGS")))
(unless (find-package "RT_DYNAMIXEL_MSGS::ERROR")
  (make-package "RT_DYNAMIXEL_MSGS::ERROR"))

(in-package "ROS")
;;//! \htmlinclude Error.msg.html


(intern "*SER_INIT_FAILED*" (find-package "RT_DYNAMIXEL_MSGS::ERROR"))
(shadow '*SER_INIT_FAILED* (find-package "RT_DYNAMIXEL_MSGS::ERROR"))
(defconstant rt_dynamixel_msgs::Error::*SER_INIT_FAILED* 1)
(intern "*OFFLINE_DETECT*" (find-package "RT_DYNAMIXEL_MSGS::ERROR"))
(shadow '*OFFLINE_DETECT* (find-package "RT_DYNAMIXEL_MSGS::ERROR"))
(defconstant rt_dynamixel_msgs::Error::*OFFLINE_DETECT* 2)
(defclass rt_dynamixel_msgs::Error
  :super ros::object
  :slots (_error_code _error_detail ))

(defmethod rt_dynamixel_msgs::Error
  (:init
   (&key
    ((:error_code __error_code) 0)
    ((:error_detail __error_detail) "")
    )
   (send-super :init)
   (setq _error_code (round __error_code))
   (setq _error_detail (string __error_detail))
   self)
  (:error_code
   (&optional __error_code)
   (if __error_code (setq _error_code __error_code)) _error_code)
  (:error_detail
   (&optional __error_detail)
   (if __error_detail (setq _error_detail __error_detail)) _error_detail)
  (:serialization-length
   ()
   (+
    ;; int32 _error_code
    4
    ;; string _error_detail
    4 (length _error_detail)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _error_code
       (write-long _error_code s)
     ;; string _error_detail
       (write-long (length _error_detail) s) (princ _error_detail s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _error_code
     (setq _error_code (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _error_detail
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _error_detail (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get rt_dynamixel_msgs::Error :md5sum-) "d97d350edf68e4d52368560032b073fd")
(setf (get rt_dynamixel_msgs::Error :datatype-) "rt_dynamixel_msgs/Error")
(setf (get rt_dynamixel_msgs::Error :definition-)
      "
int32 SER_INIT_FAILED=1
int32 OFFLINE_DETECT=2

int32 error_code
string error_detail


")



(provide :rt_dynamixel_msgs/Error "d97d350edf68e4d52368560032b073fd")


