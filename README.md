# FRC-2017
Team 254's code for the 2017 FRC Season.

CTRE Libs: http://www.ctr-electronics.com/hro.html#product_tabs_technical_resources

## Suggested Style Guide:
<table>
	<tr>
    	<th colspan=3 style="text-align:left">Objects</th>
    </tr>
    <tr>
		<th>Type</th>
		<th>Variable Name</th>
		<th>Example</th>
	</tr>
	<tr>
		<td>Hardware</td>
		<td>m_, type of hardware at the end</td>
		<td>mShooterMasterTalon<br />
		mIntakeSolenoid<br />
		mShooterRpmEncoder</td>
	</tr>
	<tr><th colspan=3 style="text-align:left">Variables</th></tr>
	<tr>
		<td>Final Constants</td>
        <td>k_</td>
        <td>kFlywheelSetpoint</td>
	</tr>
    <tr>
    	<td>Private variables</td>
        <td>m_</td>
        <td>mHeading</td>
    </tr>
    <tr><th colspan=3 style="text-align:left">Methods</th></tr>
    <tr>
      <td>Public Getter</td>
      <td>get_</td>
      <td>getDistance</td>
    </tr>
    <tr>
      <td>Public Setter</td>
      <td>set_</td>
      <td>setWantedDegrees<br />setWantToHang<br />setHeading<br />setPower<br /></td>
    </tr>
    <tr>
      <td>Public Boolean Getter</td>
      <td>is_</td>
      <td>isAbleTohang<br />isEnabled</td>
    </tr>
    <tr>
      <td colspan=3>
        <ul>
          <li>Lowercase first letter</li>
          <li>camelCase</li>
        </ul>
      </td>
    </tr>
    <tr><th colspan=3 style="text-align:left">Classes</th></tr>
    <tr>
      <td colspan=3>
        <ul>
          <li>Uppercase first letter</li>
          <li>CamelCase</li>
        </ul>
      </td>
    </tr>
    <tr><th colspan=3 style="text-align:left">Ordering Inside Files</th></tr>
    <tr>
      <td colspan=3>
      <i>Note: this is suggested, and in some cases is not the best practice</i>
        <ol>
          <li>Package Statement</li>
          <li>Imports<ol>
          	<li>WPILib/CAN Talon import statements
            <li>Other classes in the FRC-20xx Package</li>
            <li><b>Never</b> use an asterisk unless you literally are using everything in that package</li>
          </ol></li>
          <li>Class Definition</li>
          <li>Public/static/final Variables e.g. Instances of subsystems</li>
          <li>Public enums</li>
          <li>Public inner classes</li>
          <li>Private variables</li>
          <li>Private enums</li>
          <li>Constructur(s), starting with the lowest number of arguments</li>
          <li>Public methods</li>
          <li>Private methods</li>
          <li>Private inner classes</li>
        </ol>
      </td>
    </tr>
    <tr><th colspan=3 style="text-align:left">General</th></tr>
    <tr>
      <td colspan=3>
        <ul>
        <li>Curly braces on same line with space <code>if (foo == bar) {</code></li>
          <li>No one-line ifs</li>
          <li>Indentation: 4 spaces (handled by Eclipse)</li>
          <li>Empty methods - consolidate them into one line, such as: <code>public void autonomousPeriodic() { }</code></li>
        </ul>
      </td>
    </tr>
    <tr><th colspan=3 style="text-align:left">Package Structure</th></tr>
    <tr>
      <td colspan=3>
        <ul>
        <li><b>com.team254.frc2017</b><ul>
        <li><i>Core Classes (like Robot.java)</i></li>
        <li><b>_.auto</b><ul>
        <li><i>Structural autonomous classes</i></li>
        <li><b>_.actions</b> - Individual Actions for the autonomous modes</li>
        <li><b>_.modes</b> - Autonomous Modes (like cheval de frise, etc.)</li>
        </ul></li>
        <li><b>_.loops</b> - Other loops not already in a subsystem</li>
        <li><b>_.subsystems</b> - The robot's subsystems</li>
        <li><b>_.vision</b> - For vision code</li>
        </ul></li>
        <li><b>com.team254.lib</b><ul>
        <li><i>Any libraries / classes (not written by Team 254 </i></li>
        <li><b>_.util</b> - utility classes written by Team 254</li>
        </ul></li>
        </ul>
      </td>
    </tr>
</table>
