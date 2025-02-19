within Modelica.Mechanics.Rotational.Components;
model IdealCrankR2T "Ideal slider-crank mechanism transforming rotational into translational motion"
  extends Modelica.Mechanics.Rotational.Interfaces.PartialElementaryRotationalToTranslational;
  import Modelica.Math.{sin,cos,tan};

  parameter SI.Radius radius(start=1) "Crank radius";
  parameter SI.Length length(start=1) "Length of connecting rod";
  parameter SI.Distance s0 = 0 "Reference stroke";

  SI.Angle phi(start=0, stateSelect=StateSelect.default) "Crank angle to supportR";
  SI.Position s(stateSelect=StateSelect.avoid) "Relative position of flangeT to supportT";

protected
  SI.Angle alpha(stateSelect=StateSelect.never) "Connecting rod angle";
  SI.Position sp "Distance of connecting rod's translational end to crank center";
  Real ratio(final unit="m/rad") "Transmission ratio (flangeT.s/flangeR.phi)";

equation
  radius * sin(phi) = length * sin(alpha);
  ratio = -radius * (sin(phi) + cos(phi)*tan(alpha));
  sp = radius * cos(phi) + length * cos(alpha);
  sp = s + s0;

  phi = flangeR.phi - internalSupportR.phi;
  s = flangeT.s - internalSupportT.s;
  0 = ratio*flangeT.f + flangeR.tau;
  annotation (
    Documentation(info="<html>
<p>
This is an ideal, i.e. massless and inertialess, slider-crank mechanism which
transforms a&nbsp;1D-rotational into a&nbsp;1D-translational motion.
It consists of a&nbsp;crank and a&nbsp;connection rod (conrod).
The conrod&apos;s big end is connected to the crank while its small end
performs the translational motion.
If elasticity, damping or backlash has to be considered, this ideal crank
has to be connected with corresponding elements.
</p>
<p>
For <var>&phi;</var>&nbsp;= k&nbsp;*&nbsp;2*pi, k&nbsp;&isin;&nbsp;&integers;,
the conrod&apos;s small end is at the top dead center (TDC).
For <var>&phi;</var>&nbsp;= (2*k&nbsp;-&nbsp;1)*pi, k&nbsp;&isin;&nbsp;&integers;,
the small end is at the bottom dead center (BDC).
The position&nbsp;<code>s</code> is the relative position of <code>flangeT</code>
to <code>supportT</code> and is related to the reference&nbsp;<code>s0</code>
as follows.
For <code>s0</code>&nbsp;= <code>radius</code>&nbsp;&plus;&nbsp;<code>length</code>,
<code>s</code> is counted from the TDC and is, thus, always negative.
For <code>s0</code>&nbsp;= &minus;<code>radius</code>&nbsp;&plus;&nbsp;<code>length</code>,
<code>s</code> is counted from the BDC and is, thus, always positive.
Any other value of <code>s0</code> is permissible.
</p>
</html>"),
    Icon(
      coordinateSystem(preserveAspectRatio=true, extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
        Polygon(
          points={{-94,8},{-88,4},{-30,-26},{-18,-34},{-24,-44},{-38,-40},{-96,-12},{-102,-10},{-94,8}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          fillColor={131,175,131},
          fillPattern=FillPattern.Solid,
          extent={{-15,10},{15,-10}},
          origin={85,0},
          rotation=0),
        Line(points={{-100,20},{-90,20}}),
        Line(points={{-100,-20},{-90,-20}}),
        Line(points={{-100,-20},{-100,-100}}),
        Line(points={{80,-20},{100,-20},{100,-100}},
          color={0,127,0}),
        Line(points={{42,-24}}, color={255,255,255}),
        Polygon(
          points={{64,8},{52,-2},{-2,-26},{-18,-28},{-12,-48},{2,-36},{56,-12},{70,-10},{64,8}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          extent={{-8,-52},{-32,-28}}),
        Line(
          points={{80,20},{100,20}},
          color={0,127,0}),
        Ellipse(
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          extent={{80,-10},{60,10}}),
        Ellipse(
          extent={{-26,-34},{-14,-46}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{64,6},{76,-6}},
          lineColor={0,0,0},
          fillColor={131,175,131},
          fillPattern=FillPattern.Solid),
        Text(
          textColor={0,0,255},
          extent={{-150,40},{150,80}},
          textString="%name")}));
end IdealCrankR2T;
