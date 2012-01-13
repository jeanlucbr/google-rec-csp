% Copyright 2011 Google Inc. All Rights Reserved.
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

function SimulateHeliostatControl
  % SimulateHeliostatControl -- Simulates the control of heliostats.
  %
  % SimulateHeliostatControl
  % This file shows how to control of a field of heliostats.  The control
  % algorithm accounts for most of the code.  The functions that extract
  % sensor data from the accelerometer and the vision system use
  % simulated values to return realistic measurements.
  % Unless otherwise noted, all linear measurements are in meters and
  % angles in radians.

  % We define a few constants:
  % The desired convergence time, in seconds (a.k.a Tau).
  exponential_decay_time_constant = 10.0;

  % The control gain, which is used to compute the speed at which we'll
  % try to correct errors.
  control_gain = 1 / exponential_decay_time_constant;

  % The minimum interval (sampling and correction) so that the desired
  % convergence can be achieved.  Dividing by 10 means that we are giving
  % ourselves 10 corrections to converge.  10 is an engineering rule of
  % thumb.
  minimum_sampling_interval = exponential_decay_time_constant / 10.0;

  % The number of seconds between each iteration of the control loop.
  simulation_interval = 1;
  assert(simulation_interval <= minimum_sampling_interval);

  % The number of seconds per day.
  seconds_per_day = 60 * 60 * 24;

  % Set the start time and maximum end time of the simulation.
  start_time = datenum(2011, 10, 25, 6, 0, 0);
  end_time = datenum(2011, 10, 25, 19, 0, 0);
  
  % noise of accelerometer sensor (in m/sec2)
  accelerometer_noise = 0.01;
  % noise in vision sensor (m)
  vision_noise = 0.1;
  % photometry capture radius (m)
  vision_capture_radius = 3.0;
  
  % verbose run: more printout, less time duration in example
  verbose = false;
  
  % if verbose, change start and end times to shorten the run
  if (verbose)
    start_time = datenum(2011, 10, 25, 7, 57, 0);
    end_time = datenum(2011, 10, 25, 8, 0, 0);
    accelerometer_noise = 0.0;
    vision_noise = 0.0;
  end
  
  % Invoke the control algorithm.  The first parameter specifies the goal.
  % Uncomment the simulation that you want to run.
  % ControlHeliostats('Go To Protective Orientation', ...
  %                   start_time, end_time, 0, 0);
  % ControlHeliostats('Go To Cleaning Orientation', ...
  %                   start_time, end_time, 0, 0);
  % ControlHeliostats('Go To Specific Orientation', ...
  %                   start_time, end_time, 20 * pi / 180, 35 * pi / 180);
  % ControlHeliostats('Track Ideal Orientation', ...
  %                   start_time, end_time, 0, 0);
  % ControlHeliostats('Track Using Vision', start_time, end_time, 0, 0);
  ControlHeliostats('Fully Automated', start_time, end_time, 0, 0);


  function ControlHeliostats(mode, start_time, end_time, desired_pitch, ...
      desired_roll)
    % ControlHeliostats -- Controls a field of heliostats
    %
    % ControlHeliostats(start_time, mode, desired_pitch, desired_roll)
    %
    % Arguments
    %
    % mode: A string representing the control mode for the field.  The
    %       possible values are:
    %       'Stop': Stop moving.
    %       'Go To Cleaning Orientation': Go to an orientation where the
    %           heliostat can be cleaned.
    %       'Go To Protective Orientation': Go to an orientation that
    %           protects the heliostat from high winds.
    %       'Go To Specific Orientation': Go to the orientation specified
    %           by desired_pitch and desired_roll.
    %       'Track Ideal Orientation': Track to reflect the sun onto the
    %           target using only astronomical data.
    %       'Track Ideal Orientation Rate': Track to keep the reflection of
    %           the sun at the same position using only the expected
    %           movement of the sun across the sky.
    %       'Track Using Vision': Track to keep the reflection of the sun 
    %           on the target using feedback from the vision system and
    %           also by compensating for the movement of the sun during the
    %           feedback delays.
    %       'Fully Automated': Fully Automated use of the heliostat.  This
    %           can track the sun throug multiple days.
    % start_time: Starting time of the simulation.
    % end_time: Maximum end time of the simulation.  The simulation may
    %           stop earlier if the goal has been achieved.
    % desired_pitch: The pitch to achieve if the mode is 'Go To Specific
    %                Orientation'.
    % desired_roll: The roll to achieve if the mode is 'Go To Specific
    %               Orientation'.

    % Specify the location and dimension of the target.
    target.location.longitude = -117.126516;  % Somewhere close to Barstow, CA
    target.location.latitude = 34.965671;
    target.location.altitude = 705.0;
    target.center_coordinates = [0 0 5.1];
    % Specify the orientation of the target, vertical & facing north:
    % X = -east, Y = z (up), NORMAL Z = north
    target.ctm = [-1 0 0
                   0 0 1
                   0 1 0];

    % Specify the heliostats found in the field.  The first parameter of
    % CreateHeliostat specifies the type of heliostat, the second its location.
    heliostats = [CreateHeliostat('alpha', [-8.74 48.8 2.1]) ...  
                  %CreateHeliostat('junior', [-8.74 48.8 2.1]), ...
                  %CreateHeliostat('junior', [-7.523 48.8 2.1]), ...
                  %CreateHeliostat('proto3', [5.617 35.383 2.92])
                 ];

    max_duration = end_time - start_time;
    max_iterations = ceil(max_duration * seconds_per_day / ...
                          simulation_interval);

    % Allocate space for the data we collect for the graph.
    graph_time = zeros(1, max_iterations);
    graph_pitch = zeros(1, max_iterations);
    graph_roll = zeros(1, max_iterations);
    
    % Initialize
    graph_time(1) = start_time;
    graph_pitch(1) = heliostats(1).simulated_pitch;
    graph_roll(1) = heliostats(1).simulated_roll;
    goal_achieved = false;
    current_time = start_time;
    i = 1;

    % Control loop.  We do one iteration each simulation_interval seconds.
    while not(goal_achieved) && i <= max_iterations
      % fprintf('--------------%s ', datestr(current_time));
      [goal_achieved heliostats] = UpdateHeliostats(current_time, mode, ...
          target, heliostats, desired_pitch, desired_roll);    
      heliostats = SimulateMovementOfHeliostats(heliostats);
      current_time = current_time + simulation_interval / seconds_per_day;
      i = i + 1;
      
      graph_time(i) = current_time;
      graph_pitch(i) = heliostats(1).simulated_pitch;
      graph_roll(i) = heliostats(1).simulated_roll;
    end
    
    % Plot the pitch and roll of the first u-joint over time.
    graph_time = (graph_time(1 : i) - start_time) * 24;
    plot(graph_time, graph_pitch(1 : i) * 180 / pi, ...
         graph_time, graph_roll(1 : i) * 180 / pi);
    title('Pitch and Roll of the U-Joint')
    xlabel('Elapsed Time (hours)')
    ylabel('Angle (degrees)')
    handle = legend('Pitch', 'Roll');
    legend(handle, 'Location', 'BestOutside')
    
    if (verbose)
      datestr(current_time)
      fprintf('pitch = %f degrees', heliostats(1).simulated_pitch * 180 / pi);
      fprintf(' roll = %f degrees\n', heliostats(1).simulated_roll * 180 / pi);
    end
  end


  function heliostat = CreateHeliostat(type, relative_location)
    % CreateHeliostat -- Initializes one heliostat structure.
    %
    % heliostat = CreateHeliostat(type, relative_location)
    % Creates the structure we use to keep track of all the information
    % we know about a heliostat.
    %
    % Arguments
    %
    % type: A string, the type of heliostat.  Different types will have
    %       different geometries.  Legal values are 'proto3', 'junior',
    %       and 'alpha'.
    % relative_location: The relative location of the heliostat to the
    %                    target, using (East North Up) measurements.
    %
    % Return Values
    %
    % heliostat: The initialized structure.

    % These variables are established once at initialization.  They should
    % not change.

    % Set the measurements which are specific to a type of heliostat.
    % See the diagram found in section 5.1 of
    % http://www.google.org/pdfs/google_heliostat_pitch_roll_control.pdf.
    switch type
    case 'proto3'
      % These are in reflector coordinates (x, y, z):
      heliostat.mirror_pivot_to_mirror_center = [0 -0.09 0.408];  % d
      heliostat.mirror_pivot_to_mirror_attachment1 = [1.04 0.6 0.09];  % d1
      heliostat.mirror_pivot_to_mirror_attachment2 = [-1.01 0.6 0.09];  % d2
      % These are in field coordinates (East, North, Up).
      heliostat.mirror_pivot_to_ground_attachment1 = [0.655 1.02 -1.19];
      heliostat.mirror_pivot_to_ground_attachment2 = [-0.655 1.02 -1.19];
      % The angle offset of the mirror from the U-joint.
      heliostat.mirror_offset_angle = 0;
    case 'junior'
      heliostat.mirror_pivot_to_mirror_center = [0 -0.10 0.12];
      heliostat.mirror_pivot_to_mirror_attachment1 = [0.425 0.283 0.095];
      heliostat.mirror_pivot_to_mirror_attachment2 = [-0.425 0.283 0.095];
      heliostat.mirror_pivot_to_ground_attachment1 = [0.255 0.50 -0.65];
      heliostat.mirror_pivot_to_ground_attachment2 = [-0.255 0.50 -0.65];
      heliostat.mirror_offset_angle = 0;
    case 'alpha'
      heliostat.mirror_pivot_to_mirror_center = [0 -0.0763 0.217];
      heliostat.mirror_pivot_to_mirror_attachment1 = [0.979 0.618 0.110];
      heliostat.mirror_pivot_to_mirror_attachment2 = [-0.979 0.618 0.110];
      heliostat.mirror_pivot_to_ground_attachment1 = [0.563 1.047 -1.311];
      heliostat.mirror_pivot_to_ground_attachment2 = [-0.563 1.047 -1.311];
      heliostat.mirror_offset_angle = 15 * pi / 180;  % 15 degrees
    otherwise
      assert(false)
    end
    
    % Set the field coordinates of the mirror pivot point.
    heliostat.mirror_pivot_field_coordinates = relative_location;
    % Compute the ground attachment point of the actuators in field
    % coordinates.  See the diagram found in section 5.1 of
    % http://www.google.org/pdfs/google_heliostat_pitch_roll_control.pdf.
    heliostat.actuator1_ground_attachment_point = ...
        heliostat.mirror_pivot_field_coordinates + ...
        heliostat.mirror_pivot_to_ground_attachment1;  % g1
    heliostat.actuator2_ground_attachment_point = ...
        heliostat.mirror_pivot_field_coordinates + ...
        heliostat.mirror_pivot_to_ground_attachment2;  % g2
    % Set the mirror normals to be achieved when trying to protect the
    % mirror or while trying to clean it.
    heliostat.mirror_normal_when_protected = [0 0 1];  % Flat horizontal.
    heliostat.mirror_normal_when_cleaning = ...  % 70 degree pitch, 0 roll.
        PitchAndRollToCartesianNormal(70 * pi / 180, 0);

    % These variables are computed every time the orientation changes.
    % Initialization to NaN is not really required.

    % Computed mirror pitch and roll from the accelerometer data.
    heliostat.mirror_pitch = 0;
    heliostat.mirror_roll = 0;
    % Heliostat mirror coordinate transformation matrix.
    heliostat.ctm = [NaN NaN NaN
                     NaN NaN NaN
                     NaN NaN NaN];
    % Mirror center in field coordinates.
    heliostat.mirror_center_field_coordinates = [NaN NaN NaN];

    % The following data is recomputed at each iteration of the control
    % loop.
    
    % The current sun unit vector.
    heliostat.sun_unit = [NaN NaN NaN];         % s in documentation
    % The mirror normal we want to achieve.
    heliostat.ideal_mirror_normal = [NaN NaN NaN];
    % The measured/estimated mirror normal
    heliostat.mirror_normal = [NaN NaN NaN];	% h in documentation
    % The length of each actuator.
    heliostat.estimated_actuators = [NaN NaN];
    % The unit vectors for each actuator.
    heliostat.estimated_cable_unit_1 = [NaN NaN NaN];
    heliostat.estimated_cable_unit_2 = [NaN NaN NaN];
    % The rate at which we want to rotate the mirror.
    heliostat.target_angle_rates = [NaN NaN];
    % The rate at which we'll move the two actuators.
    heliostat.target_linear_rates = [NaN NaN];
    % The position of the spot on target as reported by the vision system.
    % If the vision system can't an estimate, the value will be [NaN Nan].
    heliostat.target_error = [NaN NaN];
    % The numerical derivative for ideal heliostat body angular velocity.
    heliostat.rate_feedforward = [NaN NaN NaN];
    % We keep track of the last computed ideal mirror normal.  By comparing
    % it with the new normal and taking into account how much time has
    % elapsed, we can compute cheaply the Earth's rotation for the
    % feedforward mechanism.  This assumes a consistent sampling.
    heliostat.previous_ideal_mirror_normal = [0 0 0];
    heliostat.time_of_previous_mirror_normal = 0;
    % The last time vision reported a good measurement.
    heliostat.time_of_last_good_vision = 0;

    % These values are used only so that we can simulate a real heliostat.
    % We start with the mirror at a 60 degrees pitch.
    heliostat.simulated_linear_speed = [0 0];
    heliostat.simulated_pitch = pi / 3;
    heliostat.simulated_roll = 0;
  end


  function [goal_achieved, heliostats] = UpdateHeliostats(current_time, ...
        control_mode, target, heliostats, desired_pitch, desired_roll)
    % UpdateHeliostats -- Controls the heliostats for one iteration.
    %
    % [goal_achieved, heliostats] = UpdateHeliostats(current_time, ...
    %    control_mode, target, heliostats, desired_pitch, desired_roll)
    % Controls the heliostats for one iteration of the control loop. 
    %
    % Arguments
    %
    % current_time: The start time for this iteration of the control loop.
    % control_mode: The goal to achieve.
    % target: The structure describing the target.
    % heliostats: The array of heliostat structures.
    % desired_pitch: The pitch to achieve if the mode is 'Go To Specific
    %                Orientation'.
    % desired_roll: The roll to achieve if the mode is 'Go To Specific
    %               Orientation'.
    %
    % Return Values
    %
    % goal_achieved: True if all heliostats have achieved the goal.
    % heliostats: The updated array of heliostats.

    % Compute the sun position for the current time.
    sun_position = GetSunPosition(target.location, current_time);

    % Run a simulation step for each heliostat.
    goal_achieved = true;
    for i = 1 : size(heliostats)
      [heliostat_done, heliostats(i)] = UpdateHeliostat(heliostats(i), ...
          target, current_time, sun_position, control_mode, ...
          desired_pitch, desired_roll);
      goal_achieved = goal_achieved && heliostat_done;
    end
  end


  function [goal_achieved, heliostat] = UpdateHeliostat(heliostat, ...
        target, current_time, sun_position, control_mode, ...
        desired_pitch, desired_roll)
    % UpdateHeliostat -- Controls one heliostat for one iteration.
    %
    % [goal_achieved, heliostats] = UpdateHeliostat(heliostat, ...
    %    target, current_time, sun_position, control_mode, ...
    %    desired_pitch, desired_roll)
    % Controls one heliostats for one iteration of the control loop. 
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % target: The structure describing the target.
    % current_time: The start time for this iteration of the control loop.
    % sun_position: The current position of the sun in the sky.
    % control_mode: The goal to achieve.
    % desired_pitch: The pitch to achieve if the mode is 'Go To Specific
    %                Orientation'.
    % desired_roll: The roll to achieve if the mode is 'Go To Specific
    %               Orientation'.
    %
    % Return Values
    %
    % goal_achieved: True if the heliostat has achieved the goal.
    % heliostat: The updated heliostat structure.

    heliostat.sun_unit = sun_position;
    % If needed, ask the vision system for the location of the heliostat's
    % reflection.
    if strcmp(control_mode, 'Track Using Vision') || ...
        strcmp(control_mode, 'Fully Automated')
      heliostat.target_error = ...
          GetLocationOfSpotOnTarget(heliostat, target, sun_position);
    else
      heliostat.target_error = [NaN NaN];
    end

    heliostat = DetermineCurrentOrientation(heliostat);
    heliostat = DetermineIdealMirrorNormal(heliostat, target, current_time);

    current_control_mode = control_mode;
    % If on Fully Automated control, decide what should be done based on
    % circumstances.
    if strcmp(control_mode, 'Fully Automated')
      current_control_mode = GetCurrentControlModeForAutopilot(...
          heliostat, current_time);
    end

    % Determine what should be the rates for this iteration.
    heliostat.target_angle_rates = [0 0];
    heliostat.target_linear_rates = [0 0];

    if (verbose)
      fprintf( 'ideal normal   %1.3f %1.3f %1.3f\n', ...
        heliostat.ideal_mirror_normal(1), ...
        heliostat.ideal_mirror_normal(2), ...
        heliostat.ideal_mirror_normal(3));
      fprintf( 'control mode %s    pitch %2.3f   roll %2.3f   %1.3f %1.3f %1.3f\n', ...
        current_control_mode, ...
        heliostat.simulated_pitch * 180 / pi, heliostat.simulated_roll * 180 / pi, ...
        heliostat.mirror_normal(1), heliostat.mirror_normal(2), ...
        heliostat.mirror_normal(3));
    end

    switch (current_control_mode)
    case 'Stop'
      % Target rates already set to zero, nothing to do.
    case 'Go To Protective Orientation'
      heliostat = ControlToSpecificOrientation(heliostat, ...
          heliostat.mirror_normal_when_protected, true);
    case 'Go To Cleaning Orientation'
      heliostat = ControlToSpecificOrientation(heliostat, ...
          heliostat.mirror_normal_when_cleaning, true);
    case 'Go To Specific Orientation'
      heliostat = ControlToSpecificOrientation(heliostat, ...
          PitchAndRollToCartesianNormal(desired_pitch, desired_roll), ...
          true);
    case 'Track Ideal Orientation'
      % Move to the ideal orientation based solely on time & location.
      % Don't do anything if the sun is below the horizon.
      if heliostat.sun_unit(3) > 0
        heliostat = ControlToSpecificOrientation(heliostat, ...
            heliostat.ideal_mirror_normal, false);
      end
    case 'Track Ideal Orientation Rate'
      % Keep the same position, compensating for the Earth's movement.
      heliostat = ControlOrientationRate(heliostat, ...
                                         heliostat.rate_feedforward);
    case 'Track Using Vision'
      if max(isnan(heliostat.target_error)) == 0
        % Move according to feedback and feedforward data.
        heliostat = ControlUsingFeedback(heliostat, target);
      end
    case 'Fully Automated'
      % Fail if we arrive here.  GetCurrentControlModeForAutopilot should have
      % found the correct control mode for a fully automated behavior.
      assert(0);
    otherwise
      assert(0);
    end

    % Tell the motor controls the speed to use for this iteration.
    heliostat = SetCableLengthRates(heliostat, ...
                                    heliostat.target_linear_rates(1), ...
                                    heliostat.target_linear_rates(2));

    % For non-tracking modes, we're done once we have given the command to
    % stop, i.e. rate is 0.
    mode_completes_on_no_movement = ...
        strcmp(control_mode, 'Stop') || ...
        strcmp(control_mode, 'Go To Protective Orientation') || ...
        strcmp(control_mode, 'Go To Cleaning Orientation') || ...
        strcmp(control_mode, 'Go To Specific Orientation');
    goal_achieved = mode_completes_on_no_movement && ...
        0 == heliostat.target_linear_rates(1) && ...
        0 == heliostat.target_linear_rates(2);
  end


  function heliostat = DetermineCurrentOrientation(heliostat)
    % DetermineCurrentOrientation -- Update the orientation data.
    %
    % heliostat = DetermineCurrentOrientation(heliostat)
    % Get the accelerometer data and update the structure elements
    % that track the orientation of the heliostat.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    %
    % Return Values
    %
    % heliostat: The updated heliostat structure.

    acceleration = GetAccelerometerData(heliostat);

    [heliostat.mirror_pitch heliostat.mirror_roll] = ...
        GetMirrorPitchAndRollFromAcceleration(acceleration);

    heliostat = UpdateGeometryFromPitchAndRoll(heliostat);
  end


  function [pitch roll] = GetMirrorPitchAndRollFromAcceleration(...
      acceleration)
    % GetMirrorPitchAndRollFromAcceleration -- Compute pitch and roll.
    %
    % [pitch roll] = GetMirrorPitchAndRollFromAcceleration(heliostat,
    %     acceleration)
    % Compute the mirror pitch and roll from the data returned by the
    % accelerometer.   We assume the accelerometer is mounted aligned
    % with the mirror-side of the articulation U-joint
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % acceleration: The acceleration vector returned by the accelerometer.
    %
    % Return Values
    %
    % pitch: The pitch angle of the mirror.
    % roll: The roll angle of the mirror.
    pitch = asin(-acceleration(2));
    roll = atan2(acceleration(1), -acceleration(3));
  end


  function acceleration = GetAccelerationFromMirrorPitchAndRoll(...
        mirror_pitch, mirror_roll)
    % GetAccelerationFromMirrorPitchAndRoll -- Compute acceleration.
    %
    % acceleration = GetAccelerationFromMirrorPitchAndRoll(...
    %    heliostat, mirror_pitch, mirror_roll)
    % This function simulates what the accelerometer sensor would read for
    % a given mirror pitch and roll.  We assume the accelerometer is mounted
    % aligned with the mirror-side of the articulation U-joint
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % mirror_pitch: The pitch angle of the mirror.
    % mirror_roll: The roll angle of the mirror.
    %
    % Return Values
    %
    % acceleration: The acceleration vector returned by the accelerometer.
    acceleration = [cos(mirror_pitch) * sin(mirror_roll) ...
                           -sin(mirror_pitch) ...
                           -cos(mirror_pitch) * cos(mirror_roll)];
  end


  function heliostat = UpdateGeometryFromPitchAndRoll(heliostat)
    % UpdateGeometryFromPitchAndRoll -- Updates the geometry variables.
    %
    % heliostat = UpdateGeometryFromPitchAndRoll(heliostat)
    % Updates the variables of the heliostat structure for the newly
    % set heliostat.pitch and heliostat.roll.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    %
    % Return Values
    %
    % heliostat: The updated heliostat structure.
    z_reflector = PitchAndRollToCartesianNormal(heliostat.mirror_pitch, ...
                                             heliostat.mirror_roll);

    % With the reflector z unit vector, get the heliostat orientation.
    heliostat.ctm = ...
        ComputeReflectorCoordinateTransformationMatrix(z_reflector);
    assert(norm(heliostat.ctm) > 0.9);
    assert(NearEquals(cross(heliostat.ctm(:, 1)', heliostat.ctm(:, 2)), ...
                      heliostat.ctm(:, 3)', ...
                      0.0001));

    % Compute the mirror normal for delta angle droop.
    % See heliostat orientation estimation document, equation (16)
    % mirror_normal = cos() * z - sin() * y
    delta = heliostat.mirror_offset_angle;
    heliostat.mirror_normal = cos(delta) * heliostat.ctm(:,3)' - ...
                              sin(delta) * heliostat.ctm(:,2)';
    
    % Compute the mirror center, in field coordinates
    heliostat.mirror_center_field_coordinates = ... % d in documentation
        heliostat.mirror_pivot_field_coordinates + ...
        ConvertToField(heliostat, heliostat.mirror_pivot_to_mirror_center);

    % Compute actuator connection points on the mirror, in field coordinates
    mirror_attachment1_field_coordinates = ...      % d1 in documentation
        heliostat.mirror_pivot_field_coordinates + ...
        ConvertToField(heliostat, ...
                       heliostat.mirror_pivot_to_mirror_attachment1);
    mirror_attachment2_field_coordinates = ...      % d2 in documentation
        heliostat.mirror_pivot_field_coordinates + ...
        ConvertToField(heliostat, ...
                       heliostat.mirror_pivot_to_mirror_attachment2);

    % Compute the estimated actuator unit vectors u1, u2 and estimated actuator
    % lengths s1, s2.  See figure in section 5.1 of
    % http://www.google.org/pdfs/google_heliostat_pitch_roll_control.pdf.
    s1u1 = mirror_attachment1_field_coordinates - ...
         heliostat.actuator1_ground_attachment_point;
    s2u2 = mirror_attachment2_field_coordinates - ...
         heliostat.actuator2_ground_attachment_point;
    heliostat.estimated_actuators(1) = norm(s1u1);       % s1
    heliostat.estimated_actuators(2) = norm(s2u2);       % s2
    heliostat.estimated_cable_unit_1 = s1u1 / norm(s1u1); % u1
    heliostat.estimated_cable_unit_2 = s2u2 / norm(s2u2); % u2
  end


  function ctm = ComputeReflectorCoordinateTransformationMatrix(...
      z_reflector )
    % ComputeReflectorCoordinateTransformationMatrix -- Computes the CTM.
    %
    % ctm = ComputeReflectorCoordinateTransformationMatrix(...
    %     heliostat_unit_normal)
    % For a given reflector z vector, compute the conversion matrix that
    % can be used to convert from reflector coordinates to field
    % coordinates.  To use the CTM, use the expression (ctm * vector')'.
    %
    % Arguments
    %
    % z_reflector: The unit normal of the heliostat.  The mirror
    %              normal is not the same as z_h if there is an
    %              offset (droop) angle.
    %
    % Return Values
    %
    % ctm: The CTM matrix.

    assert(abs(1 - norm(z_reflector)) < 0.001);
    roll = asin(z_reflector(1));
    pitch = -atan2(z_reflector(2), z_reflector(3));
    z_h = z_reflector;
    x_h = [cos(roll) sin(roll) * sin(pitch) -sin(roll) * cos(pitch)];
    y_h = [0 cos(pitch) sin(pitch)];
    ctm = [x_h' y_h' z_h'];
    assert(abs(norm(ctm) - 1) < 0.01);
  end


  function heliostat = DetermineIdealMirrorNormal(heliostat, target, ...
                                                current_time)
    % DetermineIdealMirrorNormal -- Get ideal heliostat pointing direction.
    %
    % heliostat = DetermineIdealMirrorNormal(heliostat, target, current_time)
    % Compute the ideal mirror normal vector.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % target: The structure describing the target.
    % current_time: The current time.
    %
    % Return Values
    %
    % heliostat: The updated heliostat structure.

    % Based on the sun position provided, compute the mirror normal so that
    % the reflection of the sun is on the target.
    assert(norm(heliostat.mirror_center_field_coordinates) > 0);
    % The heliostat normal should bisect the sun to mirror and the target
    % to mirror vectors.
    mirror_to_target = target.center_coordinates - ...
        heliostat.mirror_center_field_coordinates;
    mirror_to_target_unit = mirror_to_target / norm(mirror_to_target);

    % The middle between two unit vectors is the desired normal.
    heliostat_normal = mirror_to_target_unit + heliostat.sun_unit;
    heliostat.ideal_mirror_normal = ...
        heliostat_normal / norm(heliostat_normal);

    % Keep track of the ideal normal from tick to tick, to approximate the
    % Earth's rotation cheaply.
    if (heliostat.time_of_previous_mirror_normal > 0)
      elapsed_seconds = ...
        (current_time - heliostat.time_of_previous_mirror_normal) * ...
        seconds_per_day;
      heliostat.rate_feedforward = ...
          -1 * cross(heliostat.ideal_mirror_normal, ...
                     heliostat.previous_ideal_mirror_normal) / ...
          elapsed_seconds;
    else
      heliostat.rate_feedforward = [0 0 0];
    end
    heliostat.previous_ideal_mirror_normal = heliostat.ideal_mirror_normal;
    heliostat.time_of_previous_mirror_normal = current_time;
  end


  function control_mode = GetCurrentControlModeForAutopilot(heliostat, ...
      current_time)
    % GetCurrentControlModeForAutopilot -- Returns the appropriate mode.
    %
    % control_mode = GetCurrentControlModeForAutopilot(heliostat, ...
    %     current_time)
    % Returns the control mode for the heliostat for this iteration.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % current_time: The current time.
    %
    % Return Values
    %
    % control_mode: The control mode to follow.

    % If night time, stop.
    if (heliostat.sun_unit(3) < 0.2)
      control_mode = 'Stop';
      return;
    end
    % If dusk, go to a safe night position.
    if (heliostat.sun_unit(3) < 0.3)
      control_mode = 'Go To Cleaning Orientation';
      return;
    end

    % Has vision acquired target?
    if (max(isnan(heliostat.target_error)) == 0)
      % The vision system is working: use feedback control.
      heliostat.time_of_last_good_vision = current_time;
      control_mode = 'Track Using Vision';
      return;
    end

    % No vision: use feedforward-only for a minute, i.e. we compensate
    % only for the Earth's movement.  This hopefully will get us past
    % short interruptions.
    if (current_time - heliostat.time_of_last_good_vision <  ...
        (60 / seconds_per_day))
      control_mode = 'Track Ideal Orientation Rate';
      return;
    end

    % If it has been too long without vision data, we'll simply track
    % where the sun ought to be.  A safer system would likely move the
    % mirror off target at some point.
    control_mode = 'Track Ideal Orientation';
  end


  function heliostat = ControlToSpecificOrientation(heliostat, ...
      desired_mirror_normal, stop_when_near)
    % ControlToSpecificOrientation -- Control to achieve an orientation.
    %
    % heliostat = ControlToSpecificOrientation(heliostat, ...
    %  desired_mirror_normal, stop_when_near)
    % Sets target_angle_rates and target_linear_rates to achieve the
    % desired orientation.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % desired_mirror_normal: The normal of the mirror we are aiming to.
    % stop_when_near: If true, the rates will be set to zero once we are
    %                 very near the desired orientation.
    %
    % Return Values
    %
    % heliostat: The updated heliostat structure.

    % Compute the desired angular rate.
    h_current = heliostat.mirror_normal;
    desired_angle_change = cross(h_current, desired_mirror_normal);

    if (verbose )
      fprintf('angle lag = %3.2f\n', norm(desired_angle_change)*180/pi );
    end
    
    % If we are close enough, stop movement.
    minimum_angle_delta = 0.002;
    if stop_when_near && ...
        abs(desired_angle_change(1)) < minimum_angle_delta && ...
        abs(desired_angle_change(2)) < minimum_angle_delta
      heliostat.target_linear_rates = [0 0];
      heliostat.target_angle_rates = [0 0];
      return;
    end

    % Otherwise, control for a specific orientation rate.
    desired_angular_rate = control_gain * desired_angle_change;
    heliostat = ControlOrientationRate(heliostat, desired_angular_rate);
  end


  function heliostat = ControlOrientationRate(heliostat, ...
      desired_angular_rate)
    % ControlOrientationRate -- Control for an orientation rate.
    %
    % heliostat = ControlOrientationRate(heliostat, desired_angular_rate)
    % Sets target_angle_rates and target_linear_rates to achieve the
    % desired angular rate.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % desired_angular_rate: The desired angular rate.
    %
    % Return Values
    %
    % heliostat: The updated heliostat structure.

    % Translate the desired angular rate into local angle rates.
    heliostat.target_angle_rates = ...
        DecomposeAngularVelocity(heliostat.ctm, desired_angular_rate);
    % Convert into actuator rates.
    X = MatrixX(heliostat);
    heliostat.target_linear_rates = (X * heliostat.target_angle_rates')';
  end


  function heliostat = ControlUsingFeedback(heliostat, target)
    % ControlUsingFeedback -- Control using vision feedback.
    %
    % heliostat = ControlUsingFeedback(heliostat, target)
    % Sets target_angle_rates and target_linear_rates to correct the
    % specified target error.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % target: The structure describing the target.
    %
    % Return Values
    %
    % heliostat: The updated heliostat structure.

    % First Order Control Law: desired velocity on target from error.
    desired_x_y_speeds_on_target = -control_gain * heliostat.target_error';

    % Construct the feedback component.
    psi = MatrixPsi(heliostat, target);
    feedback_angle_rates = (inv(psi) * desired_x_y_speeds_on_target)';
    X = MatrixX(heliostat);
    feedback_linear_rates = (X * feedback_angle_rates')';

    % Construct feedforward component.
    feedforward_angle_rates = DecomposeAngularVelocity(heliostat.ctm, ...
        heliostat.rate_feedforward);
    feedforward_linear_rates = (X * feedforward_angle_rates')';

    heliostat.target_angle_rates = ...
        feedback_angle_rates + feedforward_angle_rates;
    heliostat.target_linear_rates = ...
        feedback_linear_rates + feedforward_linear_rates;
  end


  function heliostat_angular_rates = DecomposeAngularVelocity(ctm, omega)
    % DecomposeAngularVelocity -- Get an angular rate.
    %
    % heliostat_angular_rates = DecomposeAngularVelocity(ctm, omega)
    % Converts an angular velocity vector in field coordinates to the
    % corresponding heliostat angular rates.
    %
    % Arguments
    %
    % ctm: The coordinate transformation matrix.
    % omega: The angular velocity vector.
    %
    % Return Values
    %
    % heliostat_angular_rates: The corresponding heliostat angular rates.
    heliostat_angular_rates = [omega * ctm(:, 1) omega * ctm(:, 2)];
  end


  function X = MatrixX(heliostat)
    % MatrixX -- Computes the Chi matrix.
    %
    % X = MatrixX(heliostat, ctm)
    % Compute the Chi (X) matrix for this heliostat orientation.
    % This maps angular rates to linear actuator rates, i.e.
    % [ds1/dt ds2/dt] = X [pitch_rate roll_rate]
    % See the diagram found in section 5.1 of
    % http://www.google.org/pdfs/google_heliostat_pitch_roll_control.pdf.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    %
    % Return Values
    %
    % X: The Chi matrix.

    x = [1 0 0];
    y_h = heliostat.ctm(:, 2)';

    % Get the actuator unit vector directions.
    u1 = heliostat.estimated_cable_unit_1;
    u2 = heliostat.estimated_cable_unit_2;
    m1 = heliostat.ctm * heliostat.mirror_pivot_to_mirror_attachment1';
    m2 = heliostat.ctm * heliostat.mirror_pivot_to_mirror_attachment2';
    x_1_pitch = cross(x, m1) * u1';
    x_1_roll  = cross(y_h, m1) * u1';
    x_2_pitch = cross(x, m2) * u2';
    x_2_roll  = cross(y_h, m2) * u2';
    X = [x_1_pitch x_1_roll
         x_2_pitch x_2_roll];
  end



  function Psi = MatrixPsi(heliostat, target)
    % MatrixPsi -- Returns the Psi matrix.
    %
    % Psi = MatrixPsi(heliostat, target)
    % Compute the Psi matrix for this heliostat and sun orientation.
    % This maps angular rates to on-target speeds (in target coordinate
    % system), i.e.
    % [v_x v_y] = psi [pitch_rate roll_rate]
    % See the diagram found in section 5.1 of
    % http://www.google.org/pdfs/google_heliostat_pitch_roll_control.pdf.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % target: The structure describing the target.
    %
    % Return Values
    %
    % Psi: The Psi matrix.

    % Handy unit vectors: mirror normal h, target normal n.
    h = heliostat.mirror_normal;
    n = target.ctm(:, 3)';
    % Direction r to target from equation (7.1).
    r = -1 * heliostat.sun_unit + 2 * (heliostat.sun_unit * h') * h;
    % Distance to the target
    mirror_to_target = target.center_coordinates - ...
                       heliostat.mirror_center_field_coordinates;
    l = (mirror_to_target * n') / (r * n');

    % Heliostat body unit vectors along which pitch and roll are measured.
    x = [1 0 0];
    y_h = heliostat.ctm(:, 2)';
    % Equation (4.4a), (4.4b), compute on-target velocities for unit angular
    % speeds.
    p = ConvertToField(heliostat, heliostat.mirror_pivot_to_mirror_center);
    v_x = vd(x, h, l, n, r, p, heliostat.sun_unit);
    v_y_h = vd(y_h, h, l, n, r, p, heliostat.sun_unit);
    % Equation (7.16a,b)   Psi = T_inverse * [v'(w=x)  v'(w=y_h)]
    t_inv = inv(target.ctm);
    % Return only the x, y components on the target plane.
    % We could check that the z component approximates zero.
    Psi = t_inv(1:2,:) * [v_x' v_y_h'];
  end


  function v = vd(omega, h, l, n, r, p, sun_unit)
    cross_omega_h = cross(omega, h);
    dr_dt = 2 * (sun_unit * cross_omega_h') * h + ...
            2 * (sun_unit * h') * cross_omega_h;
    % Equation (7.10)
    r_times_n = r * n';
    dr_dt_times_n = dr_dt * n';
    k = l / r_times_n;
    vr = k * ((r_times_n * dr_dt) - dr_dt_times_n);
    % Equation (7.8b)
    cross_omega_p = cross(omega, p);
    vp = cross_omega_p - r * (cross_omega_p * n' / r_times_n);
    % Equation (7.8b)
    v = vr + vp;
  end


  function position = GetSunPosition(location, time)
    % GetSunPosition -- Returns the sun position vector
    %
    % position = GetSunPosition(location, time)
    % Determines the sun orientation for a given place and location.
    %
    % Arguments
    %
    % location: The location (longitude, latitude, and altitude) of the
    %           observer
    % time: The time of the observation.
    %
    % Return Values
    %
    % position: The position, as an (East North Up) vector.

    % Get the sun angle for this moment.  Use your favorite implementation.
    % There is one available at 
    % http://www.mathworks.com/matlabcentral/fileexchange/4605-sunposition-m
    datetime = datevec(time);
    tick.year = datetime(1);
    tick.month = datetime(2);
    tick.day = datetime(3);  
    tick.hour = datetime(4);
    tick.min = datetime(5);
    tick.sec = datetime(6);
    tick.UTC = -8;   % This will vary depending on your location.
    sun = sun_position(tick, location);

    [sx sy sz] = sph2cart((90 - sun.azimuth) * pi / 180, ...
                          (90 - sun.zenith) * pi / 180, 1);
    position = [sx sy sz];
  end


  function cartesian = PitchAndRollToCartesianNormal(pitch, roll)
    % PitchAndRollToCartesianNormal -- Convert to cartesian coordinates.
    %
    % cartesian = PitchAndRollToCartesianNormal(pitch, roll)
    % Convert pitch and roll to cartesian coordinates.
    %
    % Arguments
    %
    % pitch: The pitch angle.
    % roll: The roll angle.
    %
    % Return Values
    %
    % cartesian: The cartesian equivalent.
    cartesian = [sin(roll) -cos(roll) * sin(pitch) cos(roll) * cos(pitch)];
  end


  function are_equals = NearEquals(a, b, precision)
    % NearEquals -- Check near equality of two matrices.
    %
    % are_equals = NearEquals(a, b, precision)
    % Returns true if all elements of a equal those of b, for a given
    % precision.
    %
    % Arguments
    %
    % a: First matrix to compare.
    % b: Second matrix to compare.
    % precision: The precision required.
    %
    % Return Values
    %
    % are_equals: True if every element of the matrices are within
    %             precision of each other.
    %
    % We do max twice to get a scalar.
    are_equals = max(max(abs(a - b))) < precision;
  end


  function field_coordinates = ConvertToField(heliostat, ...
      heliostat_coordinates)
    % ConvertToField -- Convert heliostat coordinates to field coordinates.
    %
    % field_coordinates = ConvertToField(heliostat, heliostat_coordinates)
    % Converts heliostat coordinates to field coordinates.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % heliostat_coordinates: The coordinates to convert.
    %
    % Return Values
    %
    % field_coordinates: The converted coordinates.
    field_coordinates = (heliostat.ctm * heliostat_coordinates')';
  end


  function accelerometer_reading = GetAccelerometerData(heliostat)
    % GetAccelerometerData -- Read the accelerometer.
    %
    % accelerometer_reading = GetAccelerometerData(heliostat)
    % In a real control system, we would query the accelerometer.  Here
    % we simulate getting the accelerometer measurements.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    %
    % Return Values
    %
    % accelerometer_reading: The acceleration vector.

    % For the simulation, we use the geometry of the heliostat to
    % compute a reasonable acceleration value.
    accelerometer_reading = GetAccelerationFromMirrorPitchAndRoll(...
        heliostat.simulated_pitch, heliostat.simulated_roll);

    % Simulate a certain level of noise in the measurements
    accelerometer_reading = ...
       accelerometer_reading +  (.5 - rand(1, 3)) * accelerometer_noise;
    accelerometer_reading = ...
        accelerometer_reading / norm(accelerometer_reading);
  end


  function x_y = GetLocationOfSpotOnTarget(heliostat, target, sun_position)
    % GetLocationOfSpotOnTarget -- Get location of spot from vision system.
    %
    % x_y = GetLocationOfSpotOnTarget(heliostat, target, sun_position)
    % Simulate a vision system that from camera pictures returns the x, y
    % coordinates of the reflection of the sun on the target for that
    % heliostat.
    % 
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % target: The structure describing the target.
    % sun_position: The current sun position.  This should not be needed
    %               for a normal control system but is required for the
    %               simulation.
    %
    % Return Values
    % 
    % x_y: The [x y] coordinates in the target plane of the spot.  If the
    %      vision system can't determine it, [NaN NaN] is returned.  This
    %      could be the case if the sun is obscured by clouds.
    simulated_heliostat = UpdateGeometryFromPitchAndRoll(heliostat);
    x_y = SpotOnTarget(target.ctm, target.center_coordinates, ...
        simulated_heliostat.mirror_center_field_coordinates, ...
        simulated_heliostat.mirror_normal, sun_position);
    % Simulate a certain level of noise in the measurements
    x_y = x_y + (.5 - rand(1, 2)) * vision_noise;
    
    % If we're more than 3 meters from the target, simulate the vision
    % system unable to get any data.
    if max(abs(x_y)) > vision_capture_radius
      x_y = [NaN NaN];
    end
  end


  function heliostat = SetCableLengthRates(heliostat, left_motor_rate, ...
      right_motor_rate)
    % SetCableLengthRates -- Set the motor rates.
    %
    % heliostat = SetCableLengthRates(heliostat, left_motor_rate, ...
    %     right_motor_rate)
    % Simulate setting the cable rates.  Normally this function would send
    % commands to the motors.
    %
    % Arguments
    %
    % heliostat: The structure describing the heliostat.
    % left_motor_rate: The left motor rate, in meter / second.
    % right_motor_rate: The right motor rate, in meter / second.
    %
    % Return Values
    %
    % heliostat: The updated heliostat structure.
    
    % Simulate that the motors have a maximum speed (in meters/sec).
    maximum_speed = 0.05;
    actual_left_motor_rate = ...
        min(abs(left_motor_rate), maximum_speed) * sign(left_motor_rate);
    actual_right_motor_rate = ...
        min(abs(right_motor_rate), maximum_speed) * sign(right_motor_rate);
    heliostat.simulated_linear_speed = ...
        [actual_left_motor_rate actual_right_motor_rate];
  end


  function heliostats = SimulateMovementOfHeliostats(heliostats)
    % SimulateMovementOfHeliostats -- Simulate the heliostats moving.
    %
    % heliostats = SimulateMovementOfHeliostats(heliostats)
    % Simulate the heliostats moving during the interval by adjusting the
    % simulated_pitch and simulated_roll variables.  This function should
    % be called only once per simulation step.
    %
    % heliostats: The array of heliostat structures.
    %
    % Return Values
    %
    % heliostats: The updated array of heliostats.

    for i = 1 : size(heliostats)
      heliostat = heliostats(i);

      % Convert linear speed into angle rate.
      X = MatrixX(heliostat);
      angle_rates = (inv(X) * heliostat.simulated_linear_speed')';
      angle_changes = angle_rates * simulation_interval;

      % Crude approximation: update the mirror orientation
      heliostats(i).simulated_pitch = ...
          heliostats(i).simulated_pitch + angle_changes(1);
      heliostats(i).simulated_roll = ...
          heliostats(i).simulated_roll + angle_changes(2);
    end  
  end

  function x_y = SpotOnTarget(target_units, c, p, mirror_normal, sun_unit)
    % SpotOnTarget -- Computes the location of the spot on the target.
    %
    % x_y = SpotOnTarget(target_units, c, p, mirror_normal, sun_unit)
    % Computes the location (i.e. center) of the light spot cast by the
    % heliostat on the target, in target coordinates.
    %
    % Arguments
    %
    % target_units: The target unit vectors [x_t' y_t' z_t'].
    % c: The target center (in field coordinates).
    % p: The ray to target starting point (center of mirror).
    % mirror_normal: The mirror normal vector (in field coordinates).
    % sun_unit: The sun unit vector (to sun) (in field coordinates).
    %
    % Return Values
    %
    % x_y: The [x y] coordinates on the target plane.

    % Compute the reflected ray direction.
    ray_unit = - sun_unit + 2*(sun_unit*mirror_normal')*mirror_normal;

    target_normal = target_units(:,3)';

    % Find the intercept on the target plane.
    %   Find the line "d" on target plane that connects the plane center to
    %   the intersection point.  The intersection point is along ray of
    %   light (origin p + along ray_unit vector).  The equations:
    %
    %   center_plane + d  =  p + k * ray_unit                         (1)
    %   - dot product with plane normal cancels out "d"
    %   center_plane * normal = p * normal + k * ray_unit * normal;   (2)
    %   - solving for k
    %   k = ((c - p) * normal) / (ray_unit * normal);                 (3)
    %   - and substituting into (1) yields d, intercept vector in plane:
    %   d =  p - center_plane + k * ray_unit                         (1)
    %
    k = ((c - p) * target_normal') / (ray_unit*target_normal');
    d = ( p - c ) + k * ray_unit;

    % Do coordinate transform to express spot's center in target
    % coordinates.
    off_center = (inv( target_units ) * d')';
    x_y = off_center(1:2);
  end
end
