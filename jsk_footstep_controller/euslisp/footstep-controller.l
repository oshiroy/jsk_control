#!/usr/bin/env roseus

;; The most simplest version to execute footsteps by :set-foot-steps
;; method and have actionlib server interface
(ros::roseus "footstep_controller")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(load "package://jsk_footstep_controller/euslisp/util.l")
;; (load "package://jsk_robot_utils/euslisp/robot-interface-utils.l")
(setq *robot-name* (ros::get-param "/robot/type" (unix::getenv "ROBOT")))
(load (robot-interface-file *robot-name*))
(init-robot-from-name *robot-name*)

(defun footstep->coords (f)
  (let ((c (ros::tf-pose->coords (send f :pose))))
    (cond ((= (send f :leg) jsk_footstep_msgs::Footstep::*LEFT*)
           (send c :name :lleg))
          ((= (send f :leg) jsk_footstep_msgs::Footstep::*RIGHT*)
           (send c :name :rleg)))
    c))

(defun footstep-array->coords (arr)
  (mapcar #'(lambda (f)
              (footstep->coords f))
          (send arr :footsteps)))
(defun print-readable-coords (coords)
  (dolist (c coords)
    (format t "(make-coords :pos ~A :rpy ~A)~%"
            (send c :worldpos)
            (car (send c :rpy-angle)))))

(defun execute-cb (server goal &key(devide? t))
  (ros::ros-info "execute-cb")
  (let ((footstep (send goal :goal :footstep)))
    (ros::publish "/footstep_goal" footstep)
    ;; footstep := jsk_recognition_msgs/FootstepArray
    (let ((footstep-coords (footstep-array->coords footstep)))
      (print-readable-coords footstep-coords)
      (if devide?
	  (devide-set-foot-steps footstep-coords)
	(send *ri* :set-foot-steps footstep-coords))
      (send server :set-succeeded (send server :result))
      )))

(defun devide-set-foot-steps (footstep-coords)
  (let ((devided-foot-coords nil)
	(current-leg-coords nil)
	(pre-leg-coords nil)
	(height-threshold 200)
	(error-threshold 30)
	(foot-mid-coords (send (send *robot* :foot-midcoords) :copy-worldcoords))
	(tfc nil)
	(stair-flg nil))
    (if (equal (send (car footstep-coords) :name) ':lleg)
	(progn (setq current-leg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
	       (setq pre-leg-coords (send *robot* :lleg :end-coords :copy-worldcoords)))
      (progn (setq current-leg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
	       (setq pre-leg-coords (send *robot* :rleg :end-coords :copy-worldcoords))))
    (dolist (fc footstep-coords)
      (setq tfc (make-coords :pos (v- (send fc :pos)
				      (send foot-midcoords :pos))
			     :rpy (send fc :rpy)
			     :name (send fc :name)))
      (if stair-flg
	  (if (> error-threshold (abs (- (elt (send tfc :pos) 2) (elt (send pre-leg-coords :pos) 2))))
	      (progn 
		(setq devided-foot-coords (append devided-foot-coords (list tfc)))
		(send *ri* :set-foot-steps devided-foot-coords)
		(send *ri* :set-gait-generator-param :default-orbit-type :cycloid)
		(setq foot-midcoords (midcoords 0.5 tfc current-leg-coords))
		(ros::ros-info "set gait generator param :cycloid" (ros::get-name))
		(setq stair-flg nil)
		(setq devided-foot-coords tfc)
		)
	    (if devided-foot-coords
		(setq devided-foot-coords (append devided-foot-coords (list tfc)))
	      (setq devided-foot-coords tfc))
	    )
	(progn
	  (if (> (abs (elt (send tfc :pos) 2)) (+ (elt (send pre-leg-coords :pos) 2) hegiht-threshold))
	      (progn
		(when (> (length devided-foot-coords) 1)
		  (send *ri* :set-foot-steps devided-foot-coords)
		  (setq foot-midcoords (midcoords 0.5 current-leg-coords pre-leg-coords)))
		(send *ri* :set-gait-generator-param :default-orbit-type :stair)
		(ros::ros-info "set gait generator param :stair" (ros::get-name))
		(setq stair-flg t)
		(setq devided-foot-coords (list cuurent-leg-coords tfc))
		)
	    (if devided-foot-coords
		(setq devided-foot-coords (append devided-foot-coords (list tfc)))
	      (setq devided-foot-coords tfc)))))
      (setq pre-leg-coords current-leg-coords)
      (setq current-leg-coords tfc)
      )
    )
  )

(ros::advertise "/footstep_goal" jsk_footstep_msgs::FootstepArray)

(setq *server* (instance ros::simple-action-server :init
                         (ros::get-name)
                         jsk_footstep_msgs::ExecFootstepsAction
                         :execute-cb 'execute-cb))

(ros::rate 10)
(ros::ros-info "footstep controller is ready as ~A" (ros::get-name))
(while (ros::ok)
  (send *server* :worker)
  (ros::spin-once)
  (ros::sleep))

