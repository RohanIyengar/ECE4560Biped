%================================ Biped ===============================
%
% @class    Biped
%
% @brief    Model implementation for bipedal robots & supporting utilities 
%
%  
%  Skeleton code with stub'd out functions - to be completed as part of 
%  ECE 4560 weekly course assignments.
%
%  Note: Should utilize SE2 Matlab class
%
%================================ Biped ===============================

%
% @file     Biped.m
%
% @author   Alex H. Chang,   alexander.h.chang@gatech.edu
% @date     2017/09/26 [created]
%
% @note
%   set indent to 2 spaces.
%   set tab to 4 spaces, with conversion.
%
%================================ Biped ===============================

classdef Biped < handle
  
  
  properties
    gRO = SE2();       % robot spatial pose
    torsoSE2 = SE2();
    alphaL = []; % current biped joint configuration (angles) for left foot
    alphaR = []; % current biped joint configuration (angles) for right foot
    linkLeft = [];
    linkRight = [];
    stance = ''
    COM = [];
    motor_mass = 53.5;
    num_motor = 6;
  end
  
  %)
  %
  %============================ Member Functions ===========================
  %
  %(
  methods
    
    % Constructor
    function obj = Biped( )
        % TO DO: Assign default values for any class properties that do
        %        not otherwise have associated 'setter' class methods
        obj.torsoSE2 = SE2([0, 0], 0);
        obj.gRO = SE2([0, 0], 0);
        %Initialize Biped with 0 angles on all joints
        obj.alphaL = [0 0 0];
        obj.alphaR = [0 0 0];
        obj.stance = 'TORSO';
    end
    
    % Set geometrical properties of the biped model (eg. link lengths)
    function set_geometry( obj, a_geom )
        % TO DO: Create class properties corresponding to geometric 
        %        features of your biped; take in values of these features, 
        %        here (eg. a_geom), and assign to appropriate class 
        %        properties
        obj.linkLeft = a_geom(1,:);
        obj.linkRight = a_geom(2,:);
        obj.COM = obj.com('TORSO');
    end
    
    % Set joint configuration of the robot
    function set_alpha( obj, a_alpha )
        % TO DO: Update class property holding current joint configuration
        %        (ie. joint angles)
        obj.alphaL = a_alpha(1,:);
        obj.alphaR = a_alpha(2,:);
    end
    
    function set_stance(obj, a_stance)
        obj.stance = a_stance;
    end
    
    function set_gRO(obj, trans)
        resultMat = trans * obj.gRO;
        transNow = resultMat.getTranslation();
        obj.gRO = SE2([transNow(1) + .0001, 0], resultMat.getTheta());
    end
    
    function out = get_gRO(obj)
        out = obj.gRO;
    end
        
    % Return SE(2) poses representing left foot frame & right foot frame,
    % both relative to the torso frame 
    % (ie. g^t_lf and g^t_rf, respectively)
    function [g_t_lf, g_t_rf] = fk_torso_foot( obj )
      g_t_lf = []; g_t_rf = [];
      
      % TO DO: Compute forward kinematics (FK) of the left and right foot
      %        frames, relative to the torso frame, based on the current
      %        joint configuration (angles) of the biped
      g_torso_p1L = SE2([0, -obj.linkLeft(1)], obj.alphaL(1));
      g_p1L_p2L = SE2([0, -obj.linkLeft(2)], obj.alphaL(2));
      g_p2L_Lf = SE2([0, -obj.linkLeft(3)], obj.alphaL(3));
      g_t_lf = g_torso_p1L * g_p1L_p2L * g_p2L_Lf;
      
      g_torso_p1R = SE2([0, -obj.linkRight(1)], obj.alphaR(1));
      g_p1R_p2R = SE2([0, -obj.linkRight(2)], obj.alphaR(2));
      g_p2R_Rf = SE2([0, -obj.linkRight(3)], obj.alphaR(3));
      g_t_rf = g_torso_p1R * g_p1R_p2R * g_p2R_Rf;
    end
    
    function plot_limb(obj, tf1, tf2)
        if (isa(tf1, 'SE2')) 
            tf1 = tf1.getM();
        end
        if (isa(tf2, 'SE2'))
            tf2 = tf2.getM();
        end
        p1 = tf1 * [0;0;1];
        p2 = tf1 * tf2(:,3);
        plot([p1(1) p2(1)], [p1(2) p2(2)], 'r-')
    end
    
    %Input: Reference frame ('TORSO', 'LEFT_FOOT', 'RIGHT_FOOT')
    %frame1 and frame2 will be 2 frames relative to the input frame
    
    %If the input is 'TORSO', biped frames contains
    %{g_t_p1L, g_t_p2L, g_t_lf, g_t_lfBack, g_t_lfFront, g_t_p1R, g_t_p2R,
    %g_t_rf, g_t_rfBack, g_t_rfFront
    
    %If the input is 'LEFT_FOOT', biped frames contains
    %{g_lf_lfBack, g_lf_lfFront, g_lf_p2L, g_lf_p1L, g_lf_t, g_lf_p1R,
    %g_lf_p2R, g_lf_rf, g_lf_rfBack, g_lf_rfFront}
    
    %If the input is 'RIGHT_FOOT', biped frames contains
    %{g_rf_rfBack, g_rf_rfFront, g_rf_p2R, g_rf_p1R, g_rf_t, g_rf_p1L,
    %g_rf_p2L, g_rf_lf, g_rf_lfBack, g_rf_lfFront}
    
    function [ frame1, frame2, biped_frames ] = fwd_kinematics( obj, a_ref_frame )
        [g_t_lf, g_t_rf] = fk_torso_foot(obj);
        g_lf_t = g_t_lf.inv();
        g_rf_t = g_t_rf.inv();
        g_lf_rf = g_lf_t * g_t_rf;
        g_rf_lf = g_rf_t * g_t_lf;
        
        g_torso_p1L = obj.torsoSE2 * SE2([0, -obj.linkLeft(1)], obj.alphaL(1));
        g_p1L_p2L = SE2([0, -obj.linkLeft(2)], obj.alphaL(2));
        g_torso_p2L = g_torso_p1L * g_p1L_p2L;
        g_p2L_Lf = SE2([0, -obj.linkLeft(3)], obj.alphaL(3));
            g_Lf_footB = SE2([-obj.linkLeft(4), 0], 0);
            g_torso_lfBack = g_t_lf * g_Lf_footB;
            g_Lf_footF = SE2([obj.linkLeft(5), 0], 0);
            g_torso_lfFront = g_t_lf * g_Lf_footF;
        g_torso_p1R = obj.torsoSE2 * SE2([0, -obj.linkRight(1)], obj.alphaR(1));
        g_p1R_p2R = SE2([0, -obj.linkRight(2)], obj.alphaR(2));
        g_torso_p2R = g_torso_p1R * g_p1R_p2R;
        g_p2R_Rf = SE2([0, -obj.linkRight(3)], obj.alphaR(3));        
            g_Rf_footB = SE2([-obj.linkRight(4), 0], 0);
            g_torso_rfBack = g_t_rf * g_Rf_footB;
            g_Rf_footF = SE2([obj.linkRight(5), 0], 0);
            g_torso_rfFront = g_t_rf * g_Lf_footF;
        
        biped_frames = {};
        switch a_ref_frame
            case 'TORSO'
                frame1.name = 'g_t_lf';
                frame1.mat = g_t_lf;
                frame2.name = 'g_t_rf';
                frame2.mat = g_t_rf;
                
                frame.name = 'g_torso_p1L';
                frame.mat = g_torso_p1L;
                biped_frames{1} = frame;
                
                frame.name = 'g_torso_p2L';
                frame.mat = g_torso_p2L;
                biped_frames{2} = frame;
                
                frame.name = 'g_torso_lf';
                frame.mat = g_t_lf;
                biped_frames{3} = frame;
                
                frame.name = 'g_torso_lfBack';
                frame.mat = g_torso_lfBack;
                biped_frames{4} = frame;
                
                frame.name = 'g_torso_lfFront';
                frame.mat = g_torso_lfFront;
                biped_frames{5} = frame;
                
                frame.name = 'g_torso_p1R';
                frame.mat = g_torso_p1R;
                biped_frames{6} = frame;
                
                frame.name = 'g_torso_p2R';
                frame.mat = g_torso_p2R;
                biped_frames{7} = frame;
                
                frame.name = 'g_torso_rf';
                frame.mat = g_t_rf;
                biped_frames{8} = frame;
                
                frame.name = 'g_torso_rfBack';
                frame.mat = g_torso_rfBack;
                biped_frames{9} = frame;
                
                frame.name = 'g_torso_rfFront';
                frame.mat = g_torso_rfFront;
                biped_frames{10} = frame;
            case 'LEFT_FOOT'
                frame1.name = 'g_lf_t';
                frame1.mat = g_lf_t;
                frame2.name = 'g_lf_rf';
                frame2.mat = g_lf_rf;
                
                frame.name = 'g_lf_lfBack';
                frame.mat = g_Lf_footB;
                biped_frames{1} = frame;
                
                frame.name = 'g_lf_lfFront';
                frame.mat = g_Lf_footF;
                biped_frames{2} = frame;
                
                frame.name = 'g_lf_p2L';
                frame.mat = g_p2L_Lf.inv();
                biped_frames{3} = frame;
                
                frame.name = 'g_lf_p1L';
                frame.mat = g_p2L_Lf.inv() * g_p1L_p2L.inv();
                biped_frames{4} = frame;
                
                frame.name = 'g_lf_t';
                frame.mat = g_lf_t;
                biped_frames{5} = frame;
                
                frame.name = 'g_lf_p1R';
                frame.mat = g_lf_t * g_torso_p1R;
                biped_frames{6} = frame;
                
                frame.name = 'g_lf_p2R';
                frame.mat = g_lf_t * g_torso_p2R;
                biped_frames{7} = frame;
                
                frame.name = 'g_lf_rf';
                frame.mat = g_lf_t * g_t_rf;
                biped_frames{8} = frame;
                
                frame.name = 'g_lf_rfBack';
                frame.mat = g_lf_t * g_torso_rfBack;
                biped_frames{9} = frame;
                
                frame.name = 'g_lf_rfFront';
                frame.mat = g_lf_t * g_torso_rfFront;
                biped_frames{10} = frame;
            case 'RIGHT_FOOT'
                frame1.name = 'g_rf_t';
                frame1.mat = g_rf_t;
                frame2.name = 'g_rf_lf';
                frame2.mat = g_rf_lf;
                
                frame.name = 'g_rf_rfBack';
                frame.mat = g_Rf_footB;
                biped_frames{1} = frame;
                
                frame.name = 'g_rf_rfFront';
                frame.mat = g_Rf_footF;
                biped_frames{2} = frame;
                
                frame.name = 'g_rf_p2R';
                frame.mat = g_p2R_Rf.inv();
                biped_frames{3} = frame;
                
                frame.name = 'g_rf_p1R';
                frame.mat = g_p2R_Rf.inv() * g_p1R_p2R.inv();
                biped_frames{4} = frame;
                
                frame.name = 'g_rf_t';
                frame.mat = g_rf_t;
                biped_frames{5} = frame;
                
                frame.name = 'g_rf_p1L';
                frame.mat = g_rf_t * g_torso_p1L;
                biped_frames{6} = frame;
                
                frame.name = 'g_rf_p2L';
                frame.mat = g_rf_t * g_torso_p2L;
                biped_frames{7} = frame;
                
                frame.name = 'g_rf_lf';
                frame.mat = g_rf_t * g_t_lf;
                biped_frames{8} = frame;
                
                frame.name = 'g_rf_lfBack';
                frame.mat = g_rf_t * g_torso_lfBack;
                biped_frames{9} = frame;
                
                frame.name = 'g_rf_lfFront';
                frame.mat = g_rf_t * g_torso_lfFront;
                biped_frames{10} = frame;
            otherwise
        end
    end
            
    
    % Plot the torso frame, both foot frames and all links, consistent with
    % the biped's current joint configuration
    function plotTF( obj )
        % TO DO: Plot torso frame, both foot frames and lines representing
        %        all links of the biped
        figure
        hold on
        %Plot torso (same for left and right)
        obj.torsoSE2.plot('torso')
        %Plot first left joint
        g_torso_p1L = obj.torsoSE2 * SE2([0, -obj.linkLeft(1)], obj.alphaL(1));
        g_torso_p1L.plot('p1L')
        hold on
            %Plot line between torso and first left joint
            obj.plot_limb(obj.torsoSE2.getM(), g_torso_p1L.getM());
        %Plot second left joint
        g_p1L_p2L = SE2([0, -obj.linkLeft(2)], obj.alphaL(2));
        g_torso_p2L = g_torso_p1L * g_p1L_p2L;
        g_torso_p2L.plot('p2L')
        hold on
            %Plot line between first and second left joint
            obj.plot_limb(g_torso_p1L.getM(), g_p1L_p2L.getM());
        %Plot left foot joint
        g_p2L_Lf = SE2([0, -obj.linkLeft(3)], obj.alphaL(3));
        g_t_lf = g_torso_p1L * g_p1L_p2L * g_p2L_Lf;
        plot(g_t_lf, 't_lf')
        hold on
            %Plot line between second left joint and left foot joint
            obj.plot_limb(g_torso_p2L.getM(), g_p2L_Lf.getM());
            %Plot foot
            g_Lf_footB = SE2([-obj.linkLeft(4), 0], 0);
            obj.plot_limb(g_t_lf.getM(), g_Lf_footB.getM());
            g_Lf_footF = SE2([obj.linkLeft(5), 0], 0);
            obj.plot_limb(g_t_lf.getM(), g_Lf_footF.getM());
        %Plot first right joint
        g_torso_p1R = obj.torsoSE2 * SE2([0, -obj.linkRight(1)], obj.alphaR(1));
        g_torso_p1R.plot('p1R');
            %Plot line between torso and first right joint
            obj.plot_limb(obj.torsoSE2.getM(), g_torso_p1R.getM());
        %Plot second right joint
        g_p1R_p2R = SE2([0, -obj.linkRight(2)], obj.alphaR(2));
        g_torso_p2R = g_torso_p1R * g_p1R_p2R;
        g_torso_p2R.plot('p2R')
        hold on
            %Plot line between first and second right joint
            obj.plot_limb(g_torso_p1R.getM(), g_p1R_p2R.getM());
        %Plot right foot joint
        g_p2R_Rf = SE2([0, -obj.linkRight(3)], obj.alphaR(3));        
        g_t_rf = g_torso_p1R * g_p1R_p2R * g_p2R_Rf;
        g_t_rf.plot('t_rf')
        hold on
            %Plot line between second right joint and right foot joint
            obj.plot_limb(g_torso_p2R.getM(), g_p2R_Rf.getM());
            %Plot foot
            g_Rf_footB = SE2([-obj.linkRight(4), 0], 0);
            obj.plot_limb(g_t_rf.getM(), g_Rf_footB.getM());
            g_Rf_footF = SE2([obj.linkRight(5), 0], 0);
            obj.plot_limb(g_t_rf.getM(), g_Rf_footF.getM());
    end
    
    function plotTF2( obj )
        stance_now = obj.stance;
        [~, ~, all_frames] = obj.fwd_kinematics(stance_now);
        origin_frame = obj.gRO;
        switch stance_now
            case 'TORSO'
                % torso to right foot
                origin_frame.plot('torso');
                gT_P1L = origin_frame * all_frames{1,1}.mat;
                gT_P1L.plot('p1L');
                obj.plot_limb(origin_frame, origin_frame.inv() * gT_P1L);
                % left link 1 to left link 2
                gT_P2L = origin_frame * all_frames{1,2}.mat;
                gT_P2L.plot('p2L');
                obj.plot_limb(gT_P1L, (gT_P1L.inv() * gT_P2L));
                % left link 2 to left foot
                gT_LF = origin_frame * all_frames{1,3}.mat;
                gT_LF.plot('tLF')
                obj.plot_limb(gT_P2L, (gT_P2L.inv() * gT_LF));
                % left foot to each foot 
                gT_LFback = origin_frame * all_frames{1,4}.mat;
                gT_LFfront = origin_frame * all_frames{1,5}.mat;
                obj.plot_limb(gT_LF, gT_LF.inv() * gT_LFback);
                obj.plot_limb(gT_LF, gT_LF.inv() * gT_LFfront);
                
                % torso  to right foot
                gT_P1R = origin_frame * all_frames{1,6}.mat;
                gT_P1R.plot('p1R');
                obj.plot_limb(origin_frame, origin_frame.inv() * gT_P1R);
                % right link 1 to left link 2
                gT_P2R = origin_frame * all_frames{1,7}.mat;
                gT_P2R.plot('p2R');
                obj.plot_limb(gT_P1R, (gT_P1R.inv() * gT_P2R));
                % right link 2 to left foot
                gT_RF = origin_frame * all_frames{1,8}.mat;
                gT_RF.plot('tRF')
                obj.plot_limb(gT_P2R, (gT_P2R.inv() * gT_RF));
                % right foot to each foot 
                gT_RFback = origin_frame * all_frames{1,9}.mat;
                gT_RFfront = origin_frame * all_frames{1,10}.mat;
                obj.plot_limb(gT_RF, gT_RF.inv() * gT_RFback);
                obj.plot_limb(gT_RF, gT_RF.inv() * gT_RFfront);
                
                obj.COM = obj.com('TORSO');
            
            case 'LEFT_FOOT'
                origin_frame.plot('Left Foot')
                %plot leg lengths
                gRF_LFBack = origin_frame * all_frames{1,1}.mat;
                gLF_LFFront = origin_frame * all_frames{1,2}.mat;
                obj.plot_limb(origin_frame, origin_frame.inv() * gRF_LFBack);
                obj.plot_limb(origin_frame, origin_frame.inv() * gLF_LFFront);
                % left foot to second left link
                gLF_P2L = origin_frame * all_frames{1,3}.mat;
                gLF_P2L.plot('LF-P2L')
                obj.plot_limb(origin_frame, origin_frame.inv() * gLF_P2L);
                % second left link to first left link
                gLF_P1L = origin_frame * all_frames{1,4}.mat;
                gLF_P1L.plot('LF-P1L')
                obj.plot_limb(gLF_P2L, gLF_P2L.inv() * gLF_P1L);
                % first left link to torso
                gLF_T = origin_frame * all_frames{1,5}.mat;
                gLF_T.plot('LF-T')
                obj.plot_limb(gLF_P1L, gLF_P1L.inv() * gLF_T);
                % torso to firsr right link
                gLF_P1R = origin_frame * all_frames{1,6}.mat;
                gLF_P1R.plot('LF-P1R')
                obj.plot_limb(gLF_T, gLF_T.inv() * gLF_P1R);
                % first right link to second right link
                gLF_P2R = origin_frame * all_frames{1,7}.mat;
                gLF_P2R.plot('LF-P2R')
                obj.plot_limb(gLF_P1R, gLF_P1R.inv() * gLF_P2R);
                % second right link to right foot
                gLF_RF = origin_frame * all_frames{1,8}.mat;
                gLF_RF.plot('LF-RF')
                obj.plot_limb(gLF_P2R, gLF_P2R.inv() * gLF_RF);
                % right foot to leg lengths
                gLF_RFBack = origin_frame * all_frames{1,9}.mat;
                gLF_RFFront = origin_frame * all_frames{1,10}.mat;
                obj.plot_limb(gLF_RF, gLF_RF.inv() * gLF_RFBack);
                obj.plot_limb(gLF_RF, gLF_RF.inv() * gLF_RFFront);
                
                obj.COM = obj.com('LEFT_FOOT');
                
            case 'RIGHT_FOOT'
                origin_frame.plot('Right Foot')
                %plot leg lengths
                gRF_RFBack = origin_frame * all_frames{1,1}.mat;
                gRF_RFFront = origin_frame * all_frames{1,2}.mat;
                obj.plot_limb(origin_frame, origin_frame.inv() * gRF_RFBack);
                obj.plot_limb(origin_frame, origin_frame.inv() * gRF_RFFront);
                % right foot to second right link
                gRF_P2R = origin_frame * all_frames{1,3}.mat;
                gRF_P2R.plot('RF-P2R')
                obj.plot_limb(origin_frame, origin_frame.inv() * gRF_P2R);
                % second right link to first right link
                gRF_P1R = origin_frame * all_frames{1,4}.mat;
                gRF_P1R.plot('RF-P1R')
                obj.plot_limb(gRF_P2R, gRF_P2R.inv() * gRF_P1R);
                % first right link to torso
                gRF_T = origin_frame * all_frames{1,5}.mat;
                gRF_T.plot('RF-T')
                obj.plot_limb(gRF_P1R, gRF_P1R.inv() * gRF_T);
                % torso to first left link
                gRF_P1L = origin_frame * all_frames{1,6}.mat;
                gRF_P1L.plot('RF-P1L')
                obj.plot_limb(gRF_T, gRF_T.inv() * gRF_P1L);
                % first left link to second left link
                gRF_P2L = origin_frame * all_frames{1,7}.mat;
                gRF_P2L.plot('RF-P2L')
                obj.plot_limb(gRF_P1L, gRF_P1L.inv() * gRF_P2L);
                % second left link to left foot
                gRF_LF = origin_frame * all_frames{1,8}.mat;
                gRF_LF.plot('RF-LF')
                obj.plot_limb(gRF_P2L, gRF_P2L.inv() * gRF_LF);
                % left foot to leg lengths
                gRF_LFBack = origin_frame * all_frames{1,9}.mat;
                gRF_LFFront = origin_frame * all_frames{1,10}.mat;
                obj.plot_limb(gRF_LF, gRF_LF.inv() * gRF_LFBack);
                obj.plot_limb(gRF_LF, gRF_LF.inv() * gRF_LFFront);
                
                obj.COM = obj.com('RIGHT_FOOT');
            otherwise
                disp('Stance is incorrect');
        end
        hold on
        transNow = obj.gRO.getTranslation();
        plot(obj.COM(1) + transNow(1), obj.COM(2) + transNow(2), 'g*');
        viscircles([obj.COM(1) + transNow(1), obj.COM(2) + transNow(2)], 0.2, 'EdgeColor', 'g');
    end
    
    
    function animateTrajectory( obj, a_time, a_joint_traj )
        % Setup plot
        %figure
        %hold on;
        curr_angles = [a_joint_traj(1:3,1)'; a_joint_traj(4:6,1)'];
        obj.set_alpha(curr_angles);
        obj.plotTF2();
        currTime = a_time(1, 1);
        tocTime = 0;
        for i = 2:length(a_time)
            tic;
            curr_angles = [a_joint_traj(1:3,i)'; a_joint_traj(4:6,i)'];
            obj.set_alpha(curr_angles)
            hold off;
            clf;
            hold on;
            obj.plotTF2();
            tocTime = toc;
            timeDiff = a_time(1, i) - currTime - tocTime;
            currTime = a_time(1, i);
            pause(timeDiff);
        end
    end
    
    %Finds center of mass based on input reference frame
    %Mass of motors: 53.5g or 54.6g, need to check
    function [pos_com] = com(obj, a_ref_frame)
        [~, ~, all_frames] = obj.fwd_kinematics(a_ref_frame);
        center = [0;0;0];
        switch a_ref_frame
            case 'TORSO'
                center = center + obj.motor_mass .* (all_frames{1}.mat.getM() * [0;-0.53;1]); %p1L
                center = center + obj.motor_mass .* (all_frames{2}.mat.getM() * [0;-0.53;1]); %p2L
                center = center + obj.motor_mass .* (all_frames{3}.mat.getM() * [0.53;0;1]); %lf
                center = center + obj.motor_mass .* (all_frames{6}.mat.getM() * [0;-0.53;1]); %p1R
                center = center + obj.motor_mass .* (all_frames{7}.mat.getM() * [0;-0.53;1]); %p2R
                center = center + obj.motor_mass .* (all_frames{8}.mat.getM() * [0.53;0;1]); %rf
            case 'LEFT_FOOT'
                center = center + obj.motor_mass .* (all_frames{3}.mat.getM() * [0;-0.53;1]); %p2L
                center = center + obj.motor_mass .* (all_frames{4}.mat.getM() * [0;-0.53;1]); %p1L
                center = center + obj.motor_mass .* (all_frames{5}.mat.getM() * [0.53;0;1]); %lf
                center = center + obj.motor_mass .* (all_frames{6}.mat.getM() * [0;-0.53;1]); %p1R
                center = center + obj.motor_mass .* (all_frames{7}.mat.getM() * [0;-0.53;1]); %p2R
                center = center + obj.motor_mass .* (all_frames{8}.mat.getM() * [0.53;0;1]); %rf
            case 'RIGHT_FOOT'
                center = center + obj.motor_mass .* (all_frames{3}.mat.getM() * [0;-0.53;1]); %p2R
                center = center + obj.motor_mass .* (all_frames{4}.mat.getM() * [0;-0.53;1]); %p1R
                center = center + obj.motor_mass .* (all_frames{5}.mat.getM() * [0.53;0;1]); %rf
                center = center + obj.motor_mass .* (all_frames{6}.mat.getM() * [0;-0.53;1]); %p1L
                center = center + obj.motor_mass .* (all_frames{7}.mat.getM() * [0;-0.53;1]); %p2L
                center = center + obj.motor_mass .* (all_frames{8}.mat.getM() * [0.53;0;1]); %lf
        end
        center = center ./ (obj.num_motor * obj.motor_mass);
        pos_com = [center(1); center(2)];
    end
    
    function out = rot_mat(alpha)
        out = [cos(alpha) -sin(alpha); sin(alpha) cos(alpha)];
    end
    
    function [J] = jacobian( obj, a_alpha, a_ref_frame, a_ee_frame )
%         biped_frames = [];
%         [~,~, biped_frames] = obj.fwd_kinematics(a_ref_frame);
        J = [];
        switch a_ref_frame
            case 'TORSO'
                l1 = 0;
                l2 = 0;
                l3 = 0;
                if (strcmp(a_ee_frame, 'LEFT_FOOT'))
                    l1 = obj.linkLeft(1);
                    l2 = obj.linkLeft(2);
                    l3 = obj.linkLeft(3);
                end
                
                if (strcmp(a_ee_frame, 'RIGHT_FOOT'))
                    l1 = obj.linkRight(1);
                    l2 = obj.linkRight(2);
                    l3 = obj.linkRight(3);
                end
                syms t1 t2 t3 x(t1, t2, t3) y(t1, t2, t3) theta(t1, t2, t3)
                g_torso_LL1(t1, t2, t3) = [cos(t1), -sin(t1), 0; sin(t1), cos(t1), -l1; 0, 0, 1];
                g_LL1_LL2(t1, t2, t3) = [cos(t2), -sin(t2), 0; sin(t2), cos(t2), -l2; 0, 0, 1];
                g_LL2_LF(t1, t2, t3) = [cos(t3), -sin(t3), 0; sin(t3), cos(t3), -l3; 0, 0, 1];
                final_config(t1, t2, t3) = simplify(g_torso_LL1 * g_LL1_LL2 * g_LL2_LF);
                final_config_sym = final_config(t1, t2, t3);
                x(t1, t2, t3) = final_config_sym(1, 3);
                y(t1, t2, t3) = final_config_sym(2, 3);
                theta(t1, t2, t3) = t1 + t2 + t3;
                qe = [x; y; theta];
                J_sym = jacobian(qe,[t1, t2, t3]);
                J = double(J_sym(a_alpha(1), a_alpha(2), a_alpha(3)));
        end
           
    end
    
    function [traj_alpha] = rr_inv_kin( obj, a_traj_ws, a_time, a_ref_frame, a_ee_frame )
        [g_t_lf, g_t_rf] = fk_torso_foot(obj);
        lf_mat = g_t_lf.getM();
        traj_alpha = [];
        switch a_ref_frame
            case 'TORSO'
                if (a_ee_frame == 'LEFT_FOOT')
                    xi = lf_mat(1,3);
                    yi = lf_mat(2,3);
                    thetai = acos(lf_mat(1,1));
                    t = 0;
                    alpha1 = obj.alphaL(1);
                    alpha2 = obj.alphaL(2);
                    alpha3 = obj.alphaL(3);
                    a_alpha = [alpha1; alpha2; alpha3];
                    for i = 1:length(a_time)
                        dt = a_time(i) - t;
                        t = a_time(i);
                        x_dot = a_traj_ws(1,i) - xi;
                        y_dot = a_traj_ws(2,i) - yi;
                        theta_dot = a_traj_ws(3,i) - thetai;
                        xi = a_traj_ws(1,i);
                        yi = a_traj_ws(2,i);
                        thetai = a_traj_ws(3,i);
                        twist = [x_dot; y_dot; theta_dot] ./ dt;
                        J = jacobian(obj, a_alpha, a_ref_frame, a_ee_frame);
                        a_dot = pinv(J) * twist;
                        a_alpha = a_alpha + a_dot .* dt;
                        traj_alpha = [traj_alpha a_alpha];
                    end
                end
        end
                    
    end
  end     % methods
end
