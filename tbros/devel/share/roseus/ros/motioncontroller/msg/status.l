;; Auto-generated. Do not edit!


(when (boundp 'motioncontroller::status)
  (if (not (find-package "MOTIONCONTROLLER"))
    (make-package "MOTIONCONTROLLER"))
  (shadow 'status (find-package "MOTIONCONTROLLER")))
(unless (find-package "MOTIONCONTROLLER::STATUS")
  (make-package "MOTIONCONTROLLER::STATUS"))

(in-package "ROS")
;;//! \htmlinclude status.msg.html


(defclass motioncontroller::status
  :super ros::object
  :slots (_errorcode ))

(defmethod motioncontroller::status
  (:init
   (&key
    ((:errorcode __errorcode) "")
    )
   (send-super :init)
   (setq _errorcode (string __errorcode))
   self)
  (:errorcode
   (&optional __errorcode)
   (if __errorcode (setq _errorcode __errorcode)) _errorcode)
  (:serialization-length
   ()
   (+
    ;; string _errorcode
    4 (length _errorcode)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _errorcode
       (write-long (length _errorcode) s) (princ _errorcode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _errorcode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _errorcode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get motioncontroller::status :md5sum-) "ec7d276cbbe91ad63105ca2d4c757803")
(setf (get motioncontroller::status :datatype-) "motioncontroller/status")
(setf (get motioncontroller::status :definition-)
      "string errorcode

")



(provide :motioncontroller/status "ec7d276cbbe91ad63105ca2d4c757803")


