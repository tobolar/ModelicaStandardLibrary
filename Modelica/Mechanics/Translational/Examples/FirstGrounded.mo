within Modelica.Mechanics.Translational.Examples;
model FirstGrounded "First example: simple drive train with grounded elements"
  extends Modelica.Icons.Example;
  parameter SI.Force amplitude=10 "Amplitude of driving force";
  parameter SI.Frequency f=5 "Frequency of driving force";
  parameter SI.Mass m_motor(min=0) = 0.1 "Motor mass";
  parameter SI.Mass m_load(min=0) = 2 "Load mass";
  parameter Real ratio=10 "Gear ratio";
  parameter SI.TranslationalDampingConstant damping=10 "Damping in bearing of lever";

  Components.Fixed fixed annotation (Placement(transformation(extent={{40,-80},{60,-60}})));
  Sources.Force force(
    useSupport=false) annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  Components.Mass mass1(
    m=m_motor) annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Components.IdealLever idealLever(
    ratio=ratio,
    useSupport=false) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Components.Mass mass2(
    m=2,
    s(fixed=true, start=0),
    v(fixed=true, start=0)) annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Components.Spring spring(
    s_rel(fixed=true, start=0),
    c=1.e4)
    annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  Components.Mass mass3(m=m_load, v(fixed=true, start=0)) annotation (Placement(transformation(extent={{80,-10},{100,10}})));
  Components.Damper damper(
    d=damping) annotation (Placement(
        transformation(
        origin={50,-50},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Blocks.Sources.Sine sine(
    amplitude=amplitude,
    f=f)
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
equation
  connect(mass1.flange_b, idealLever.flange_a) annotation (Line(points={{-20,0},{-10,0}}, color={0,127,0}));
  connect(idealLever.flange_b, mass2.flange_a) annotation (Line(points={{10,0},{20,0}}, color={0,127,0}));
  connect(mass2.flange_b, spring.flange_a) annotation (Line(points={{40,0},{50,0}}, color={0,127,0}));
  connect(spring.flange_b, mass3.flange_a) annotation (Line(points={{70,0},{80,0}}, color={0,127,0}));
  connect(damper.flange_a, mass2.flange_b) annotation (Line(points={{50,-40},{50,0},{40,0}}, color={0,127,0}));
  connect(damper.flange_b, fixed.flange)
    annotation (Line(points={{50,-60},{50,-70}}, color={0,127,0}));
  connect(sine.y, force.f) annotation (Line(points={{-79,0},{-72,0}}, color={0,0,127}));
  connect(force.flange, mass1.flange_a) annotation (Line(points={{-50,0},{-40,0}}, color={0,127,0}));
  annotation (
    experiment(
      StopTime=1.0,
      Interval=0.001),
    Documentation(info="<html>
<p>
The drive train consists of a motor mass which is driven by
a sine-wave motor force. Via a lever the translational energy is
transmitted to a load mass. Elasticity in the lever is modeled
by a spring element. A linear damper is used to model the
damping in the lever bearing.
</p>
<p>
Note, that a force component (like the damper of this example)
which is acting between a shaft and the housing has to be fixed
in the housing on one side via component
<a href=\"modelica:Modelica.Mechanics.Translational.Components.Fixed\">Fixed</a>.
</p>
<p>
Simulate for 1 second and plot the velocities of masses:
<code>mass2.v</code>, <code>mass3.v</code>
</p>
</html>"));
end FirstGrounded;
