;; Auto-generated. Do not edit!


(when (boundp 'balltracker::command)
  (if (not (find-package "BALLTRACKER"))
    (make-package "BALLTRACKER"))
  (shadow 'command (find-package "BALLTRACKER")))
(unless (find-package "BALLTRACKER::COMMAND")
  (make-package "BALLTRACKER::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude command.msg.html


(defclass balltracker::command
  :super ros::object
  :slots (_priority _direction _speed _angle ))

(defmethod balltracker::command
  (:init
   (&key
    ((:priority __priority) "")
    ((:direction __direction) "")
    ((:speed __speed) 0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _priority (string __priority))
   (setq _direction (string __direction))
   (setq _speed (round __speed))
   (setq _angle (float __angle))
   self)
  (:priority
   (&optional __priority)
   (if __priority (setq _priority __priority)) _priority)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; string _priority
    4 (length _priority)
    ;; string _direction
    4 (length _direction)
    ;; int32 _speed
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _priority
       (write-long (length _priority) s) (princ _priority s)
     ;; string _direction
       (write-long (length _direction) s) (princ _direction s)
     ;; int32 _speed
       (write-long _speed s)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _priority
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _priority (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get balltracker::command :md5sum-) "42cf6ba923645af9d43237e750433163")
(setf (get balltracker::command :datatype-) "balltracker/command")
(setf (get balltracker::command :definition-)
      "string priority
string direction
int32 speed
float32 angle

")



(provide :balltracker/command "42cf6ba923645af9d43237e750433163")


