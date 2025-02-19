within Modelica.Mechanics.Rotational.Examples;
model SliderCrank "Slider-crank mechanism"
  extends Modelica.Icons.Example;

  Components.IdealCrankR2T idealCrankR2T(
    useSupportR=true,
    useSupportT=true,
    radius=0.1,
    length=0.4,
    s0=idealCrankR2T.length) annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  Components.Inertia crankshaft(
    J=0.15,
    phi(fixed=true, start=-0.00034906585039887),
    w(fixed=true, start=0)) annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Translational.Components.Mass piston(m=0.5) annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Components.Fixed fixedR annotation (Placement(
        transformation(extent={{-20,-40},{0,-20}})));
  Translational.Components.Fixed fixedT(s0=0.02) annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
  Sources.ConstantTorque constantTorque(tau_constant=14) annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Translational.Sources.QuadraticSpeedDependentForce quadraticSpeedDependentForce(
    f_nominal=-20,
    ForceDirection=false,
    v_nominal=1) annotation (Placement(transformation(extent={{90,-10},{70,10}})));
equation
  connect(crankshaft.flange_b, idealCrankR2T.flangeR) annotation (Line(points={{-20,0},{0,0}}));
  connect(idealCrankR2T.flangeT, piston.flange_a) annotation (Line(points={{20,0},{40,0}}, color={0,127,0}));
  connect(idealCrankR2T.supportR,fixedR. flange) annotation (Line(
      points={{0,-10},{0,-30},{-10,-30}}));
  connect(fixedT.flange, idealCrankR2T.supportT) annotation (Line(points={{30,-30},{20,-30},{20,-10}}, color={0,127,0}));
  connect(constantTorque.flange, crankshaft.flange_a) annotation (Line(points={{-60,0},{-40,0}}, color={0,0,0}));
  connect(piston.flange_b, quadraticSpeedDependentForce.flange) annotation (Line(points={{60,0},{70,0}}, color={0,127,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=2,
      __Dymola_NumberOfIntervals=5001,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));
end SliderCrank;
