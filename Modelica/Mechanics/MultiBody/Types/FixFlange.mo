within Modelica.Mechanics.MultiBody.Types;
type FixFlange = enumeration(
    none
       "None of flanges fixed (housing is given outside)",
    support
       "Fix support flange",
    axis
       "Fix axis flange",
    by3DRoot
       "Flange fixing given by 3D rooting") "Enumeration to define the 1D flange which shall be fixed"
  annotation (Documentation(info="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\">Caption starts with a capital letter</caption>
  <tr>
    <th>Types.FixFrame.</th>
    <th>Meaning</th>
  </tr>
  <tr>
    <td>none</td>
    <td>Neither support nor axis flange is fixed internally, a&nbsp;housing shall be given outside</td>
  </tr>
  <tr>
    <td>support</td>
    <td>Support flange is fixed</td>
  </tr>
  <tr>
    <td>axis</td>
    <td>Axis flange is fixed</td>
  </tr>
  <tr>
    <td>by3DRoot</td>
    <td>Either support or axis flange is fixed, depending on which 3D frame is rooted</td>
  </tr>
</table>
</html>"));
