within Modelica.Mechanics.MultiBody.Joints;
model Spherical
  "Spherical joint (3 constraints and no potential states, or 3 degrees-of-freedom and 3 states)"

  import Modelica.Mechanics.MultiBody.Frames;

  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  parameter Boolean animation=true
    "= true, if animation shall be enabled (show sphere)";
  parameter SI.Distance sphereDiameter=world.defaultJointLength
    "Diameter of sphere representing the spherical joint"
    annotation (Dialog(group="if animation = true", enable=animation));
  input Types.Color sphereColor=Modelica.Mechanics.MultiBody.Types.Defaults.JointColor
    "Color of sphere representing the spherical joint"
    annotation (Dialog(colorSelector=true, group="if animation = true", enable=animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient
    "Reflection of ambient light (= 0: light is completely absorbed)"
    annotation (Dialog(group="if animation = true", enable=animation));

  parameter Boolean angles_fixed = false
    "= true, if angles_start are used as initial values, else as guess values"
    annotation(Evaluate=true, choices(checkBox=true), Dialog(tab="Initialization"));
  parameter SI.Angle angles_start[3]={0,0,0}
    "Initial values of angles to rotate frame_a around 'sequence_start' axes into frame_b"
    annotation (Dialog(tab="Initialization"));
  parameter Types.RotationSequence sequence_start={1,2,3}
    "Sequence of rotations to rotate frame_a into frame_b at initial time"
    annotation (Evaluate=true, Dialog(tab="Initialization"));

  parameter Boolean w_rel_a_fixed = false
    "= true, if w_rel_a_start are used as initial values, else as guess values"
    annotation(Evaluate=true, choices(checkBox=true), Dialog(tab="Initialization"));
  parameter SI.AngularVelocity w_rel_a_start[3]={0,0,0}
    "Initial values of angular velocity of frame_b with respect to frame_a, resolved in frame_a"
    annotation (Dialog(tab="Initialization"));

  parameter Boolean z_rel_a_fixed = false
    "= true, if z_rel_a_start are used as initial values, else as guess values"
    annotation(Evaluate=true, choices(checkBox=true), Dialog(tab="Initialization"));
  parameter SI.AngularAcceleration z_rel_a_start[3]={0,0,0}
    "Initial values of angular acceleration z_rel_a = der(w_rel_a)"
    annotation (Dialog(tab="Initialization"));

  parameter Boolean enforceStates=false
    "= true, if relative variables of spherical joint shall be used as states (StateSelect.always)"
    annotation (Evaluate=true, Dialog(tab="Advanced"));
  parameter Boolean useQuaternions=true
    "= true, if quaternions shall be used as states otherwise use 3 angles as states (provided enforceStates=true)"
    annotation (Evaluate=true, Dialog(tab="Advanced", enable=enforceStates));
  parameter Types.RotationSequence sequence_angleStates={1,2,3}
    "Sequence of rotations to rotate frame_a into frame_b around the 3 angles used as states"
     annotation (Evaluate=true, Dialog(tab="Advanced", enable=enforceStates
           and not useQuaternions));

  final parameter Frames.Orientation R_rel_start=
      Frames.axesRotations(sequence_start, angles_start, zeros(3))
    "Orientation object from frame_a to frame_b at initial time";

protected
  Visualizers.Advanced.Shape sphere(
    shapeType="sphere",
    color=sphereColor,
    specularCoefficient=specularCoefficient,
    length=sphereDiameter,
    width=sphereDiameter,
    height=sphereDiameter,
    lengthDirection={1,0,0},
    widthDirection={0,1,0},
    r_shape={-0.5,0,0}*sphereDiameter,
    r=frame_a.r_0,
    R=frame_a.R) if world.enableAnimation and animation;

  // Declarations for quaternions (dummies, if quaternions are not used)
  parameter Frames.Quaternions.Orientation Q_start=
            Modelica.Mechanics.MultiBody.Frames.to_Q(R_rel_start)
    "Quaternion orientation object from frame_a to frame_b at initial time";
  Frames.Quaternions.Orientation Q(start=Q_start, each stateSelect=if
        enforceStates and useQuaternions then StateSelect.prefer else
        StateSelect.never)
    "Quaternion orientation object from frame_a to frame_b (dummy value, if quaternions are not used as states)";

  // Declaration for 3 angles
  parameter SI.Angle phi_start[3]=if sequence_start[1] ==
      sequence_angleStates[1] and sequence_start[2] == sequence_angleStates[2]
       and sequence_start[3] == sequence_angleStates[3] then angles_start else
       Frames.axesRotationsAngles(R_rel_start, sequence_angleStates)
    "Potential angle states at initial time";
  SI.Angle phi[3](start=phi_start, each stateSelect=if enforceStates and not
        useQuaternions then StateSelect.always else StateSelect.never)
    "Dummy or 3 angles to rotate frame_a into frame_b";
  SI.AngularVelocity phi_d[3](each stateSelect=if enforceStates and not
        useQuaternions then StateSelect.always else StateSelect.never)
    "= der(phi)";
  SI.AngularAcceleration phi_dd[3] "= der(phi_d)";

  // Other declarations
  SI.AngularVelocity w_rel[3](start=Frames.resolve2(R_rel_start, w_rel_a_start),
        fixed = fill(w_rel_a_fixed,3), each stateSelect=if
        enforceStates and useQuaternions then StateSelect.always else
        StateSelect.never)
    "Dummy or relative angular velocity of frame_b with respect to frame_a, resolved in frame_b";
  Frames.Orientation R_rel
    "Dummy or relative orientation object to rotate from frame_a to frame_b";
  Frames.Orientation R_rel_inv
    "Dummy or relative orientation object to rotate from frame_b to frame_a";
initial equation
  if angles_fixed then
    if not enforceStates then
      // no states defined in spherical object
      zeros(3) = Frames.Orientation.equalityConstraint(Frames.absoluteRotation(frame_a.R,R_rel_start),frame_b.R);
    elseif useQuaternions then
      // Quaternions Q are used as states
      zeros(3) = Frames.Quaternions.Orientation.equalityConstraint(Q, Q_start);
    else
      // The 3 angles 'phi' are used as states
      phi = phi_start;
    end if;
  end if;

  if z_rel_a_fixed then
    // Initialize acceleration variables
    der(w_rel) = Frames.resolve2(R_rel_start, z_rel_a_start);
  end if;
equation
  // torque balance
  zeros(3) = frame_a.t;
  zeros(3) = frame_b.t;

  if enforceStates then
    Connections.branch(frame_a.R, frame_b.R);

    frame_b.r_0 = frame_a.r_0;
    if Connections.rooted(frame_a.R) then
      R_rel_inv = Frames.nullRotation();
      frame_b.R = Frames.absoluteRotation(frame_a.R, R_rel);
      zeros(3) = frame_a.f + Frames.resolve1(R_rel, frame_b.f);
    else
      R_rel_inv = Frames.inverseRotation(R_rel);
      frame_a.R = Frames.absoluteRotation(frame_b.R, R_rel_inv);
      zeros(3) = frame_b.f + Frames.resolve2(R_rel, frame_a.f);
    end if;

    // Compute relative orientation object
    if useQuaternions then
      // Use Quaternions as states (with dynamic state selection)
      {0} = Frames.Quaternions.orientationConstraint(Q);
      w_rel = Frames.Quaternions.angularVelocity2(Q, der(Q));
      R_rel = Frames.from_Q(Q, w_rel);

      // Dummies
      phi = zeros(3);
      phi_d = zeros(3);
      phi_dd = zeros(3);

    else
      // Use angles as states
      phi_d = der(phi);
      phi_dd = der(phi_d);
      R_rel = Frames.axesRotations(sequence_angleStates, phi, phi_d);
      w_rel = Frames.angularVelocity2(R_rel);

      // Dummies
      Q = zeros(4);
    end if;

  else
    // Spherical joint does not have states
    frame_b.r_0 = frame_a.r_0;
    //frame_b.r_0 = transpose(frame_b.R.T)*(frame_b.R.T*(transpose(frame_a.R.T)*(frame_a.R.T*frame_a.r_0)));

    zeros(3) = frame_a.f + Frames.resolveRelative(frame_b.f, frame_b.R, frame_a.R);

    if w_rel_a_fixed or z_rel_a_fixed then
      w_rel = Frames.angularVelocity2(frame_b.R) - Frames.resolve2(frame_b.R,
         Frames.angularVelocity1(frame_a.R));
    else
      w_rel = zeros(3);
    end if;

    // Dummies
    R_rel = Frames.nullRotation();
    R_rel_inv = Frames.nullRotation();
    Q = zeros(4);
    phi = zeros(3);
    phi_d = zeros(3);
    phi_dd = zeros(3);
  end if;
  annotation (
    Documentation(info="<html>
<p>
Joint with <em>3&nbsp;constraints</em> that define that the <em>origin</em>
of <code>frame_a</code> and the <em>origin</em> of <code>frame_b</code>
coincide. By default this joint defines only the 3&nbsp;constraints
without any potential states.
If parameter <code>enforceStates</code> is set to true in the
\"Advanced\" menu, three states are introduced.
Depending on parameter <code>useQuaternions</code> these are either
<a href=\"modelica://Modelica.Mechanics.MultiBody.Frames.Quaternions\">quaternions</a>
and the relative angular velocity or three angles and the angle
derivatives. In the latter case the orientation of <code>frame_b</code>
is computed by rotating <code>frame_a</code> sequentially around the axes
defined in parameter vector <code>sequence_angleStates</code> (default = {1,2,3},
i.e., the so-called Cardan angle sequence) by the angles used as states.
For example, the default is to rotate the x-axis of frame_a
by <code>angles[1]</code>, the new y-axis by <code>angles[2]</code> and,
finally, the new z-axis by <code>angles[3]</code>, thus arriving at
<code>frame_b</code>. If angles are used as states there is the slight
disadvantage that a&nbsp;singular configuration is present leading
to a&nbsp;division by zero.
</p>
<p>
If this joint is used in a&nbsp;<em>chain</em> kinematic structure,
a&nbsp;Modelica translator
has to select orientation coordinates of a&nbsp;body as states, if the
default setting is used. It is usually better to use relative coordinates
in the spherical joint as states, and therefore in this situation
parameter <code>enforceStates&nbsp;= true</code> might be set.
</p>
<p>
If this joint is used in a&nbsp;<em>loop</em> kinematic structure,
the default setting results in a&nbsp;<em>cut-joint</em> that
breaks the loop in independent kinematic branches, hold together
by the constraints of this joint. As a&nbsp;result, a&nbsp;Modelica
translator
will first try to select three generalized coordinates in the joints of
the remaining branches of the loop and their first derivative as states,
and if this is not possible, e.g., because there are only spherical
joints in the loop, it will select coordinates from a&nbsp;body of
the loop as states.
</p>
<p>
In the following figure the animation of a&nbsp;spherical
joint is shown. The light blue coordinate system is
<code>frame_a</code> and the dark blue coordinate system is
<code>frame_b</code> of the joint.
(here: <code>angles_start&nbsp;= {45&deg;, 45&deg;, 45&deg;}<code>).
</p>

<div>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/Spherical.png\" alt=\"Animation of a spherical joint\">
</div>
</html>"),
    Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}}), graphics={
        Rectangle(
          extent={{-100,10},{100,-10}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={192,192,192}),
        Ellipse(
          extent={{-60,-60},{60,60}},
          fillPattern=FillPattern.Solid,
          fillColor={192,192,192},
          lineColor={0,0,0},
          closure=EllipseClosure.Radial,
          startAngle=60,
          endAngle=300),
        Ellipse(
          extent={{-44,-44},{44,44}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          closure=EllipseClosure.Radial,
          startAngle=55,
          endAngle=305),
        Ellipse(
          extent={{-44,-44},{44,44}},
          startAngle=60,
          endAngle=300,
          lineColor={0,0,0},
          closure=EllipseClosure.None),
        Ellipse(
          extent={{-26,26},{26,-26}},
          fillPattern=FillPattern.Sphere,
          fillColor={192,192,192},
          lineColor={0,0,0}),
        Text(
          extent={{-150,110},{150,70}},
          textString="%name",
          textColor={0,0,255})}));
end Spherical;
