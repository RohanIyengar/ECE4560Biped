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
    gRO = [];       % robot spatial pose
    torsoSE2 = SE2();
    alphaL = []; % current biped joint configuration (angles) for left foot
    alphaR = []; % current biped joint configuration (angles) for right foot
    linkLeft = [];
    linkRight = [];
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
        obj.torsoSE2 = SE2([0, 0], -pi/2);
        obj.gRO = obj.torsoSE2.getM();
        %Initialize Biped with 0 angles on all joints
        obj.alphaL = [0 0 0];
        obj.alphaR = [0 0 0];
    end
    
    % Set geometrical properties of the biped model (eg. link lengths)
    function set_geometry( obj, a_geom )
        % TO DO: Create class properties corresponding to geometric 
        %        features of your biped; take in values of these features, 
        %        here (eg. a_geom), and assign to appropriate class 
        %        properties
        obj.linkLeft = a_geom(1,:);
        obj.linkRight = a_geom(2,:);
    end
    
    % Set joint configuration of the robot
    function set_alpha( obj, a_alpha )
        % TO DO: Update class property holding current joint configuration
        %        (ie. joint angles)
        obj.alphaL = a_alpha(1,:);
        obj.alphaR = a_alpha(2,:);
    end
        
    % Return SE(2) poses representing left foot frame & right foot frame,
    % both relative to the torso frame 
    % (ie. g^t_lf and g^t_rf, respectively)
    function [g_t_lf, g_t_rf] = fk_torso_foot( obj )
      g_t_lf = []; g_t_rf = [];
      
      % TO DO: Compute forward kinematics (FK) of the left and right foot
      %        frames, relative to the torso frame, based on the current
      %        joint configuration (angles) of the biped
      g_torso_p1L = SE2([obj.linkLeft(1), 0], obj.alphaL(1));
      g_p1L_p2L = SE2([obj.linkLeft(2), 0], obj.alphaL(2));
      g_p2L_Lf = SE2([obj.linkLeft(3), 0], obj.alphaL(3));
      g_t_lf = g_torso_p1L * g_p1L_p2L * g_p2L_Lf;
      
      g_torso_p1R = SE2([obj.linkRight(1), 0], obj.alphaR(1));
      g_p1R_p2R = SE2([obj.linkRight(2), 0], obj.alphaR(2));
      g_p2R_Rf = SE2([obj.linkRight(3), 0], obj.alphaR(3));
      g_t_rf = g_torso_p1R * g_p1R_p2R * g_p2R_Rf;
    end
    
    % Plot the torso frame, both foot frames and all links, consistent with
    % the biped's current joint configuration
    function plotTF( obj )
        % TO DO: Plot torso frame, both foot frames and lines representing
        %        all links of the biped
        figure
        hold on
        obj.torsoSE2.plot('torso')
        g_torso_p1L = obj.torsoSE2 * SE2([obj.linkLeft(1), 0], obj.alphaL(1));
        g_torso_p1L.plot('p1L')
        hold on
        g_p1L_p2L = SE2([obj.linkLeft(2), 0], obj.alphaL(2));
        g_torso_p2L = g_torso_p1L * g_p1L_p2L;
        g_torso_p2L.plot('p2L')
        hold on
        g_p2L_Lf = SE2([obj.linkLeft(3), 0], obj.alphaL(3));
        g_t_lf = g_torso_p1L * g_p1L_p2L * g_p2L_Lf;
        plot(g_t_lf, 't_lf')
        hold on
      
        g_torso_p1R = obj.torsoSE2 * SE2([obj.linkRight(1), 0], obj.alphaR(1));
        g_torso_p1R.plot('p1R');
        g_p1R_p2R = SE2([obj.linkRight(2), 0], obj.alphaR(2));
        g_torso_p2R = g_torso_p1R * g_p1R_p2R;
        g_torso_p2R.plot('p2R')
        hold on
        g_p2R_Rf = SE2([obj.linkRight(3), 0], obj.alphaR(3));        
        g_t_rf = g_torso_p1R * g_p1R_p2R * g_p2R_Rf;
        g_t_rf.plot('t_rf')
        hold on
    end
  end     % methods
end

