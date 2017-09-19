% PLOTCAMERA - Plots graphical representation of camera(s) showing pose
%
% Usage plotcamera(C, l, col, plotCamPath, fig)
%
% Arguments:
%            C - Camera structure (or structure array).
%            l - The length of the sides of the rectangular cone indicating
%                the camera's field of view.
%          col - Optional three element vector specifying the RGB colour to
%                use. If omitted or empty defaults to blue.
%  plotCamPath - Optional flag 0/1 to plot line joining camera centre
%                positions. If omitted or empty defaults to 0.
%          fig - Optional figure number to be used.
%
%  The function plots into the current figure a graphical representation of one
%  or more cameras showing their pose.  This consists of a rectangular cone,
%  with its vertex at the camera centre, indicating the camera's field of view.
%  The camera's coordinate axes are also plotted at the camera centre.
%
% See also: CAMERASTRUCT

% Copyright (c) Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

% September 2009
% September 2011 Fixed long standing bug in plotting camera frame.
%                Allowance for cameras being a structure array.
% June      2015 Changes to make compatible with new version of
%                CAMERASTRUCT.

function plotcamera(C, l, col, plotCamPath, fig)

    if ~exist('col', 'var')  || isempty(col)
        col = [0 0 1];
    end
    
    if ~exist('plotCamPath', 'var') || isempty(plotCamPath)
        plotCamPath = 0;
    end    
      
    if exist('fig', 'var')    
        figure(fig)
    end
    
    for i = 1:length(C)
        
        if C(i).rows == 0 || C(i).cols == 0
            warning('Camera rows and cols not specified');
            continue
        end
        
        % If only one focal length specified in structure use it for both fx and fy
        if isfield(C, 'f')
            f = C.f;
        elseif isfield(C, 'fx') && isfield(C, 'fy')
            f = C.fx;  % Use fx as the focal length
        else
            error('Invalid focal length specification in camera structure');
        end    
        
        if i > 1 & plotCamPath
            line([C(i-1).P(1) C(i).P(1)],...
                 [C(i-1).P(2) C(i).P(2)],...
                 [C(i-1).P(3) C(i).P(3)])
        end
        
        % Construct transform from camera coordinates to world coords
        Tw_c = [C(i).Rc_w'  C(i).P
                 0 0 0        1  ];        
        
        % Generate the 4 viewing rays that emanate from the principal point and
        % pass through the corners of the image.
        corner{1} = [-C(i).cols/2; -C(i).rows/2; f];
        corner{2} = [ C(i).cols/2; -C(i).rows/2; f];
        corner{3} = [ C(i).cols/2;  C(i).rows/2; f];
        corner{4} = [-C(i).cols/2;  C(i).rows/2; f];
        
        for n = 1:4
            % Scale rays to length l and make homogeneous
            ray{n} = [corner{n}*l/f; 1];
            
            % Transform to world coords
            ray{n} = Tw_c*ray{n}; 
            
            % Draw the ray
            line([C(i).P(1), ray{n}(1)],...
                 [C(i).P(2), ray{n}(2)],...
                 [C(i).P(3), ray{n}(3)], ...
                 'color', col);        
        end
        
        % Draw rectangle joining ends of rays
        line([ray{1}(1) ray{2}(1) ray{3}(1) ray{4}(1) ray{1}(1)],...
             [ray{1}(2) ray{2}(2) ray{3}(2) ray{4}(2) ray{1}(2)],...
             [ray{1}(3) ray{2}(3) ray{3}(3) ray{4}(3) ray{1}(3)],...
             'color', col);                 
        
        % Draw and label axes
        X = Tw_c(1:3,1)*l + C(i).P;
        Y = Tw_c(1:3,2)*l + C(i).P;    
        Z = Tw_c(1:3,3)*l + C(i).P;            
        
        line([C(i).P(1), X(1,1)], [C(i).P(2), X(2,1)], [C(i).P(3), X(3,1)],...
             'color', col);        
        line([C(i).P(1), Y(1,1)], [C(i).P(2), Y(2,1)], [C(i).P(3), Y(3,1)],...
             'color', col);        
        %    line([C(i).P(1), Z(1,1)], [C(i).P(2), Z(2,1)], [C(i).P(3), Z(3,1)],...
        %         'color', col);        
        text(X(1), X(2), X(3), 'X', 'color', col);        
        text(Y(1), Y(2), Y(3), 'Y', 'color', col);        
        
    end