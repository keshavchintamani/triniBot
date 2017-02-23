;; Auto-generated. Do not edit!


(when (boundp 'balltracker::ballcoords)
  (if (not (find-package "BALLTRACKER"))
    (make-package "BALLTRACKER"))
  (shadow 'ballcoords (find-package "BALLTRACKER")))
(unless (find-package "BALLTRACKER::BALLCOORDS")
  (make-package "BALLTRACKER::BALLCOORDS"))

(in-package "ROS")
;;//! \htmlinclude ballcoords.msg.html


(defclass balltracker::ballcoords
  :super ros::object
  :slots (_color _x _y _radius ))

(defmethod balltracker::ballcoords
  (:init
   (&key
    ((:color __color) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:radius __radius) 0.0)
    )
   (send-super :init)
   (setq _color (string __color))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _radius (float __radius))
   self)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:serialization-length
   ()
   (+
    ;; string _color
    4 (length _color)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _radius
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _color
       (write-long (length _color) s) (princ _color s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _color
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _color (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _radius
     (setq _radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get balltracker::ballcoords :md5sum-) "d5c701da1a7655c4c6cd03fc0658634a")
(setf (get balltracker::ballcoords :datatype-) "balltracker/ballcoords")
(setf (get balltracker::ballcoords :definition-)
      "string color
float32 x
float32 y
float32 radius

")



(provide :balltracker/ballcoords "d5c701da1a7655c4c6cd03fc0658634a")


