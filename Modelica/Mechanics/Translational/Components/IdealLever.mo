within Modelica.Mechanics.Translational.Components;
model IdealLever "Ideal lever without mass"
  extends Modelica.Mechanics.Translational.Interfaces.PartialElementaryTwoFlangesAndSupport2;
  parameter Real ratio(start=1)
    "Transmission ratio (flange_a.s/flange_b.s)";

equation
  s_a = ratio*s_b;
  0 = ratio*flange_a.f + flange_b.f;

  annotation (
    Documentation(info="<html>
<p>
This element characterizes any type of lever which is fixed in the
ground and which has one driving flange and one driven flange.
The lever is <strong>ideal</strong>, i.e., it does not have mass, elasticity, damping
or backlash. If these effects have to be considered, the lever has to be
connected to other elements in an appropriate way.
</p>
<p>
The parameter <code>ratio</code> is the ratio between the driving flange
position and the driven flange position, i.e. <code>flange_a.s/flange_b.s</code>.
The type of lever is given by the magnitude of <code>ratio</code> as follows:
</p>
<ul>
  <li>
    <code>ratio&nbsp;&lt;&nbsp;0</code>: <em>1st class lever</em>; the lever&apos;s pivot is
    located between the driving and the driven flange.
  </li>
  <li>
    <code>ratio&nbsp;&gt;&nbsp;1</code>: <em>2nd class lever</em>; the driven
    flange is located between the driving flange and the lever&apos;s pivot.
  </li>
  <li>
    <code>0&nbsp;&lt;&nbsp;ratio&nbsp;&lt;&nbsp;1</code>: <em>3rd class lever</em>; the driving flange is
    located between the driven flange and the lever&apos;s pivot.
  </li>
</ul>
</html>"),
    Icon(
      coordinateSystem(preserveAspectRatio=true,
        extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(
          points={{-6,-28},{22,50},{30,50},{36,46},{8,-32},{-6,-28}},
          lineColor={0,0,0},
          fillColor={160,215,160},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-20,-10},{20,-50}},
          fillColor={160,215,160},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-20,-10},{20,-50}},
          startAngle=-47,
          endAngle=273,
          closure=EllipseClosure.None),
        Ellipse(
          extent={{-10,-20},{10,-40}},
          fillColor={175,190,175},
          fillPattern=FillPattern.Solid),
        Line(points={{0,-54},{0,-30}},
                                     color={0,127,0}),
        Line(points={{0,-100},{0,-78}}, color={0,127,0}),
        Text(
          extent={{-150,100},{150,60}},
          textColor={0,0,255},
          textString="%name"),
        Text(extent={{-100,-50},{100,-80}},
          textString="ratio=%ratio"),
        Line(points={{-100,0},{30,50}}, color={0,127,0}),
        Line(points={{20,20},{100,0}}, color={0,127,0}),
        Ellipse(
          extent={{26,54},{34,46}},
          lineColor={0,127,0},
          fillColor={160,215,160},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{16,24},{22,18}},
          lineColor={0,127,0},
          fillColor={160,215,160},
          fillPattern=FillPattern.Solid)}));
end IdealLever;
