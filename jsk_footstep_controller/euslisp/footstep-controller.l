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

(setq *obs-idx* -1)

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

(defun cluster-point-indices-cb (msg)
  (let ((boxel-num-array nil)
	(cluster-indices (send msg :cluster_indices)))
    (mapcar #'(lambda (i)
		(length (send i :indices))
		) cluster-indices)
  ))

(defun emergency-stop()
  (let ((remain-fs-idx (send *ri* :get-current-footstep-index)))
    (when (> (1+ remain-fs-idx) *obs-idx*)
      (send *ri* :emergency-walking-stop))))

(defun execute-cb (server goal)
  (ros::ros-info "execute-cb")
  (let ((footstep (send goal :goal :footstep)))
    (ros::publish "/footstep_goal" footstep)
    ;; footstep := jsk_recognition_msgs/FootstepArray
    (let ((footstep-coords (footstep-array->coords footstep)))
      (print-readable-coords footstep-coords)
      (setq obs-idx -1)
      (send *ri* :set-foot-steps footstep-coords)
      (send server :set-succeeded (send server :result))
      )))

(ros::advertise "/footstep_goal" jsk_footstep_msgs::FootstepArray)

(setq *server* (instance ros::simple-action-server :init
                         (ros::get-name)
                         jsk_footstep_msgs::ExecFootstepsAction
                         :execute-cb 'execute-cb))


(ros::subscribe "/footstep_attention_clipper/output/cluster_point_indices" jsk_recognition_msgs::ClusterPointIndices #'cluster-point-indices-cb)

(ros::rate 10)
(ros::ros-info "footstep controller is ready as ~A" (ros::get-name))
(while (ros::ok)
  (send *server* :worker)
  (ros::spin-once)
  (ros::sleep))

