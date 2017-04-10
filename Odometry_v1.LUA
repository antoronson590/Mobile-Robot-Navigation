-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

-- Angle Correction (V-REP -> Real System)
function angle_correction(angle)
  -- The allowed angle of rotation are 0~360. However, it is observed, the rotation is moving beyond 360.
  local new_angle = math.abs(angle*180/math.pi) - 360;
  new_angle = new_angle*(math.pi/180) * (angle/math.abs(angle));
  return new_angle
end

-- To set wheel vlocity, based on sensor reading
function wheel_velocity(input_handle, input_value,l_joint, r_joint)
  local value = (simGetUISlider(input_handle,input_value))-500	-- Provision to adjust the max wheel velocity, for turning
  local velocity = -5;
  local dt = simGetSimulationTime()
  print(dt)
  if dt >0 then											-- Max Wheel Velocity(in deg/s)
    if value<0 then												-- Left Turn
      vel_l = velocity*(500-math.abs(value))/500;
      simSetJointTargetVelocity(l_joint,vel_l);
    else
      simSetJointTargetVelocity(l_joint,velocity);
    end
    if value>0 then												-- Right Turn
      vel_r = velocity * (500-value)/500;
      simSetJointTargetVelocity(r_joint,vel_r);
    else
      simSetJointTargetVelocity(r_joint,velocity);
    end
  else
    simSetJointTargetVelocity(l_joint,0)
    simSetJointTargetVelocity(r_joint,0)
  end
end

--  To determine the distance wheel has moved for every time step
function delta_x(posi,posi_old)
  -- We calculate how much distance have it moved between the last and current position of joint
  d_theta = l_per_deg         -- Calculate from the wheel circumference.
 -- local dt = simGetSimulationTimeStep()
 -- local vel = simGetJointTargetVelocity(posi)
  d_thet = math.abs(posi - posi_old);
  local distance = d_thet*d_theta*180/math.pi;
  return distance;
end

-- Returns the parameters [new positon x, new position y, new orientation theta]
function delta_odometry(x_0,y_0,alpha_0,dx,dy)
  -- The formulas are absorbed from the research paper "A Primer on Odometry and Motion Control"
  local d_baseline = 0.116               -- The distance between wheel center
  local dcenter = (dx+dy)/2;
  local d_phi = (dy-dx)/(d_baseline);
  local alpha = alpha_0+d_phi;
  if((math.abs(alpha*180/math.pi))>360) then    -- When angle crosses 2PI
    alpha = angle_correction(alpha);	-- in rad
  end
  local x = x_0+(dcenter*math.cos(alpha_0));-- in m
  local y = y_0+(dcenter*math.sin(alpha_0));-- in m 
  return x,y,alpha;
end

function angle_determination(x_init, y_init, x_fin, y_fin)
  local radius = math.pow((math.pow((x_fin-x_init),2)+math.pow((y_fin-y_init),2)),0.5)
  local theta = math.acos((x_fin - x_init)/radius)*180/math.pi;
  return theta
end

-- Initialization block ( Variable initializaiton and component initiailziation
if (sim_call_type==sim_childscriptcall_initialization) then

  -- Slider Initialization
  input_handle = simGetUIHandle('Input_Slider');
  input_value = 3;
  input = simGetUIHandle('IP_Box')
  x_target = simGetUIButtonLabel(input,3)
  if x_target == nil then
    simSetUIButtonLabel(input,3,0)
  end
  y_target = simGetUIButtonLabel(input,4)
  if y_target == nil then
    simSetUIButtonLabel(input,4,0)
  end

  -- Circumference calculation(Constant)
  dia = 0.1;					-- Wheel Diameter[m]
  l_per_deg = math.pi*dia/360;	-- circum/deg [m/deg]

  -- Joint initialization and initial joint position determination
  l_joint = simGetObjectHandle('Joint_Left');
  r_joint = simGetObjectHandle('Joint_Right');
  l_pos_old = simGetJointPosition(l_joint);
  r_pos_old = simGetJointPosition(r_joint);
  l_pos = 0;
  r_pos = 0;


  -- Variable Initialization
  alph_0 = 0;  -- Initial Orientation of Body
  x_0 = 0       -- Initial Position - x
  y_0 = 0       -- Initial Position - y
  x = 0         -- Initialization
  y = 0;
  del_l = 0;    -- Change in left Wheel
  del_r = 0;    -- Change in right Wheel
  alph  = 0;    -- Orientation Initialization

  -- Error Plotting
  position = {};
  body = simGetObjectHandle('Body');
  position_old = simGetObjectPosition(body,-1);
  orientation = simGetObjectOrientation(body, -1)
  agraph = simGetObjectHandle('Graph_Actual');

  -- Sensor Initialization and Reading
  sensor = simGetObjectHandle('Proximity_sensor');
  s_value = 0;

  -- Method 2
  -- Initialization
  -- We calculate the body poisition vector and calculate the angle from the value shift in x and y direction.
  delt_ang = 0

  -- Angle Calculation, Assuming the point is at a distance from us in a circular area

  x_target = tonumber(simGetUIButtonLabel(input,3))
  y_target = tonumber(simGetUIButtonLabel(input,4))
  if (position_old[1] == x_target) and (position_old[2] == y_target) then -- if home & target are same stop simAddBanner
    simStopSimulation()
  end

  thet = angle_determination(position_old[1], position_old[2],x_target,y_target)  
  current_angle = 0
  test = 0
  alph_0 = 0
  pos_j = {l_joint,r_joint}
  pos_jv = {}
  pos_jv_old = {l_pos_old,r_pos_old}


end

-- Executed for the rest of the simulation
if (sim_call_type==sim_childscriptcall_actuation) then


    simSetUISlider(input_handle,input_value,500)

  s_value= simReadProximitySensor(sensor);		-- Sensor Reading[1 -> Detected Blocking, -1 -> Error, 0 -> Free]

    orientation = simGetObjectOrientation(body, -1);

  if s_value == 1 then 
    simSetUISlider(input_handle,input_value,100)
  end
  wheel_velocity(input_handle, input_value,l_joint, r_joint)	-- Set Wheel Velocity
  l_pos = simGetJointPosition(l_joint);							-- Change on joint position (left & right)
  r_pos = simGetJointPosition(r_joint);
  del_l = delta_x(l_pos,l_pos_old);										-- Distance moved (left & right)
  del_r = delta_x(r_pos,r_pos_old);
  x,y,alph = delta_odometry(x_0,y_0,orientation[3],del_l,del_r)		-- Position determination
  simSetGraphUserData(agraph,'x_value',x);
  simSetGraphUserData(agraph,'y_value',y);
  position = simGetObjectPosition(body, -1);

   
  -- Data Exchange
  l_pos_old = l_pos;
  r_pos_old = r_pos;
  x_0 = x;
  y_0 = y;
  alph_0 = alph;
  position_old = position
end


if (sim_call_type==sim_childscriptcall_sensing) then

	-- Put your main SENSING code here

end


if (sim_call_type==sim_childscriptcall_cleanup) then

	-- Put some restoration code here

end-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)
-- Angle Correction

