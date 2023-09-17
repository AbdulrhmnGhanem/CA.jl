package InvertedPendulum "Example of a linnear quadratic regulator used to stabilize a inverted pendulum"
  package Components "Components package for the InvertedPendulum example"
    extends Modelica.Icons;

    partial model BasePendulumModel "Partial inverted pendelum model"
      PlanarMechanics.Parts.Body pendulum(I = pendulumInertia, m = pendulumMass) "Pendulum mass." annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-50, -80}), visible = true));
      PlanarMechanics.Parts.Fixed origin "Center point. Reference point for cart." annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-100, 10}), visible = true));
      PlanarMechanics.Parts.Body cart(m = cartMass, I = cartInertia) "Mass of cart." annotation(Placement(transformation(extent = {{0, 20}, {20, 40}}, origin = {-40, -20}, rotation = 0), visible = true));
      inner PlanarMechanics.PlanarWorld planarWorld(animateGravity = false) annotation(Placement(transformation(extent = {{-80, -20}, {-60, 0}}, origin = {-10, -20}, rotation = 0), visible = true));
      PlanarMechanics.Joints.Revolute revolute(useFlange = true, phi.start = angle0, phi.fixed = true) "Pendulum axis of rotation." annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-50, -20}), visible = true));
      PlanarMechanics.Joints.Prismatic cartPosition(useFlange = true, r = {1, 0}, s(fixed = true, start = 0), v(fixed = true), animate = false) "Translational position of cart." annotation(Placement(transformation(extent = {{-38, 20}, {-18, 40}}, origin = {-42, -20}, rotation = 0), visible = true));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angle "Angle sensor for the pendulum." annotation(Placement(visible = true, transformation(origin = {20, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MatrixGain controller(K = {{-59.8109, -15.9831, 1., 3.03214}}) "LQ-regulator with optimal gains." annotation(Placement(visible = true, transformation(origin = {60, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Translational.Sensors.PositionSensor position "Position sensor for the cart." annotation(Placement(visible = true, transformation(origin = {20, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor angularVelocity "Velocity sensor for the pendulum." annotation(Placement(visible = true, transformation(origin = {20, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Translational.Sensors.SpeedSensor speed "Velocity sensor for the cart." annotation(Placement(visible = true, transformation(origin = {20, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      PlanarMechanics.Sources.WorldForce disturbancePendulum(N_to_m = 0.1, color = {163, 50, 79}) "Disturbance force applied on the pendulum." annotation(Placement(visible = true, transformation(origin = {-10, -70}, extent = {{10, -10}, {-10, 10}}, rotation = -360)));
      PlanarMechanics.Sources.WorldForce disturbanceCart(N_to_m = 0.1, color = {27, 89, 174}) "Disturbance force applied on the cart." annotation(Placement(visible = true, transformation(origin = {-10, -50}, extent = {{10, -10}, {-10, 10}}, rotation = -360)));
      PlanarMechanics.Parts.FixedTranslation pendulumLength(r = {0, length}, width = 1 / planarWorld.defaultWidthFraction) "Length of the pendulum." annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-50, -40}), visible = true));
      PlanarMechanics.Sources.WorldForce controllerForce(N_to_m = 0.1, color = {187, 160, 49}) "Corrective force applied to the cart." annotation(Placement(visible = true, transformation(origin = {-30, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -360)));
      parameter Modelica.Units.SI.Mass cartMass = 1 "Mass of the cart, the base of the pendulum." annotation(Dialog(group = "Pendulum Properties"));
      parameter Modelica.Units.SI.Inertia cartInertia = 0.1 "Inertia of the cart, the base of the pendulum." annotation(Dialog(group = "Pendulum Properties"));
      parameter Modelica.Units.SI.Mass pendulumMass = 0.5 "Inertia of the top of the pendulum." annotation(Dialog(group = "Pendulum Properties"));
      parameter Modelica.Units.SI.Inertia pendulumInertia = 0.1 "Inertia at the top of the pendulum." annotation(Dialog(group = "Pendulum Properties"));
      parameter Modelica.Units.SI.Angle angle0 = 0 "Initial angle of the pendulum." annotation(Dialog(group = "Pendulum Properties"));
      constant Modelica.Units.SI.Length length = 1 "Distance between the cart and the top of the pendulum." annotation(Dialog(group = "Pendulum Properties"));
      Modelica.Units.SI.Position pendelum_pos[2] = pendulum.r "transl. position (pendulum.r)";
      Modelica.Blocks.Interfaces.RealOutput s = position.s "Absolute position of flange as output signal (position.s)";
      Modelica.Blocks.Interfaces.RealOutput v = speed.v "Absolute velocity of flange as output signal (speed.v)";
      Modelica.Blocks.Interfaces.RealOutput phi = angle.phi "Absolute angle of flange as output signal (angle.phi)";
      Modelica.Blocks.Interfaces.RealOutput w = angularVelocity.w "Absolute angular velocity of flange as output signal (angularVelocity.w)";
      Modelica.Blocks.Interfaces.RealOutput y[controller.nout] = controller.y "Connector of Real output signals (controller.y)";
      Real cartDisturbance[3] = disturbanceCart.force "x-, y-coordinates of force and torque resolved in world frame (disturbanceCart.force)";
      Real pendelumDisturbance[3] = disturbancePendulum.force "x-, y-coordinates of force and torque resolved in world frame (disturbancePendulum.force)";
      Modelica.Units.SI.Acceleration a = cartPosition.a "acceleration of elongation (cartPosition.a)";
      Modelica.Units.SI.AngularAcceleration z = revolute.z "Angular acceleration (revolute.z)";
    equation
      connect(revolute.frame_a, cart.frame_a) annotation(Line(points = {{-3.333, -13.333}, {-3.333, 6.667}, {6.667, 6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-46.667, 3.333}));
      connect(origin.frame, cartPosition.frame_a) annotation(Line(points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-85, 10}));
      connect(cartPosition.frame_b, cart.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-50, 10}));
      connect(position.flange, cartPosition.flange_a) annotation(Line(visible = true, origin = {-43.333, 39.667}, points = {{53.333, 10.333}, {-26.667, 10.333}, {-26.667, -20.667}}, color = {0, 127, 0}));
      connect(revolute.flange_a, angle.flange) annotation(Line(visible = true, origin = {-2.5, -15}, points = {{-37.5, -5}, {2.5, -5}, {2.5, 5}, {12.5, 5}}, color = {64, 64, 64}));
      connect(revolute.flange_a, angularVelocity.flange) annotation(Line(visible = true, origin = {-2.5, -25}, points = {{-37.5, 5}, {2.5, 5}, {2.5, -5}, {12.5, -5}}, color = {64, 64, 64}));
      connect(cartPosition.flange_a, speed.flange) annotation(Line(visible = true, origin = {-43.333, 26.333}, points = {{-26.667, -7.333}, {-26.667, 3.667}, {53.333, 3.667}}, color = {0, 127, 0}));
      connect(position.s, controller.u[3]) annotation(Line(visible = true, origin = {42.25, 30}, points = {{-11.25, 20}, {-2.25, 20}, {-2.25, -20}, {5.75, -20}}, color = {1, 37, 163}));
      connect(speed.v, controller.u[4]) annotation(Line(visible = true, origin = {42.25, 20}, points = {{-11.25, 10}, {-2.25, 10}, {-2.25, -10}, {5.75, -10}}, color = {1, 37, 163}));
      connect(angle.phi, controller.u[1]) annotation(Line(visible = true, origin = {42.25, 0}, points = {{-11.25, -10}, {-2.25, -10}, {-2.25, 10}, {5.75, 10}}, color = {1, 37, 163}));
      connect(angularVelocity.w, controller.u[2]) annotation(Line(visible = true, origin = {42.25, -10}, points = {{-11.25, -20}, {-2.25, -20}, {-2.25, 20}, {5.75, 20}}, color = {1, 37, 163}));
      connect(pendulumLength.frame_b, pendulum.frame_a) annotation(Line(points = {{0, 10}, {0, -10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-50, -60}));
      connect(pendulumLength.frame_a, revolute.frame_b) annotation(Line(visible = true, origin = {-50, -30}, points = {{0, 0}, {0, 0}, {0, 0}}, color = {95, 95, 95}));
      connect(disturbancePendulum.frame_b, pendulum.frame_a) annotation(Line(visible = true, origin = {-32.5, -65}, points = {{12.5, -5}, {-17.5, -5}}, color = {95, 95, 95}));
      connect(disturbanceCart.frame_b, cart.frame_a) annotation(Line(visible = true, origin = {-36.667, -15}, points = {{16.667, -35}, {6.667, -35}, {6.667, 10}, {-13.333, 10}, {-13.333, 25}, {-3.333, 25}}, color = {95, 95, 95}));
      connect(controllerForce.frame_b, cart.frame_a) annotation(Line(visible = true, origin = {-45, 40}, points = {{5, 30}, {-5, 30}, {-5, -30}, {5, -30}}, color = {95, 95, 95}));
      connect(controllerForce.force[1], controller.y[1]) annotation(Line(visible = true, origin = {53.25, 40}, points = {{-71.25, 30}, {26.75, 30}, {26.75, -30}, {17.75, -30}}, color = {0, 36, 164}));
      annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, lineColor = {75, 138, 73}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Polygon(visible = true, lineColor = {0, 0, 255}, fillColor = {75, 138, 73}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-36, 60}, {64, 0}, {-36, -60}, {-36, 60}})}), Documentation(info = "<html><!--WSMINSERTIONTAGSTART InvertedPendulum.Components.BasePendulumModel -->
   <head>
   <style type=\"text/css\">
   
  body {
   padding: 0px;
   margin: 0px;
  }
  
  a {
   color: #cf1d24;
  }
  
  a.target {
   padding-top: 40px;
   margin-top: -40px;
  }
  
  p {
   font-family: arial;
   font-size: 13;
   margin: 9px 40px 5px 40px;
   padding-bottom: 0px;
   color: #555555;
   max-width: 800px;
  }
  
  h1 {
   font-size: 30;
   color: #cf1d24;
   font-weight: bold;
   margin-left: 20px;
   margin-top: 32px;
   margin-bottom: 15px;
   margin-right: 20px;
   padding-top: 0px;
  }
  
  h2 {
   font-size: 20;
   color: #cf1d24;
   font-weight: bold;
   margin-left: 20px;
   margin-right: 20px;
   margin-top: 5px;
   margin-bottom: 9px;
  }
  
  h3 {
   background: url('DocumentationFiles/dingbat3.png') no-repeat 0 0;
   font-size: 14px;
   font-family: helvetica;
   color: #4B4B4B;
   font-weight: bold;
   padding-left: 18px;
   margin-left: 20px;
   margin-right: 20px;
   margin-top: 12px;
   margin-bottom: 8px;
  }
  
  ul {
   font-family: arial;
   font-size: 13;
   margin: 9px 40px 5px 40px;
   padding-bottom: 0px;
   color: #555555;
   max-width: 800px;
   list-style-type: square;
  }
  
  li {
   margin-left: 0px;
   margin-top: 4px;
   margin-bottom: 2px;
   padding-left: 0px;
  }

  div.header {
   background: url('DocumentationFiles/WSMLogo.png') no-repeat 0 0;
   height: 96px;
   margin-top: 35px;
   background-color: #871613;
  }
  
  .headerspan {
   font-family: arial;
   text-decoration: none;
   font-size: 12px;
   font-weight: bold;
   display: inline-block;
   height: 35px;
   color: gray;
   padding: 0px;
   margin: 0px;
   margin-left: 20px;
  }
  
  .headera {
   font-family: arial;
   text-decoration: none;
   font-size: 12px;
   font-weight: bold;
   padding: 0px;
   color: inherit;
   vertical-align: middle;
   margin: 0px;
  }
  
  .headerlinkdiv {
   background: black;
   padding: 0px;
   height: 35px;
   margin: 0px;
   position: fixed; top: 0px; left: 0px; width: 100%;
  }
  
  .contenttable {
   -webkit-box-shadow: 3px 3px 3px #DDDDDD;
   border-top: 3px solid #cf1d24;
   background: #f9f9f9;
   max-width: 500px;
   margin: 15px 15px 0px 20px;
   padding: 6px 10px 3px 10px;
  }
  
  .contenttableheader {
   color: #a4a4a4;
   font-size: 14px;
   font-family: arial;
  }
  
  .contenttabletable {
   border: 0px solid #FFFFFF;
   padding: 0px;
   padding-left: 20px;
  }
  
  .contenttable tr td {
   padding: 3px;
   min-width: 200px;
  }
  
  .contenttable tr td a {
   color: #555555;
   text-decoration: none;
   font-size: 13px;
   font-family: arial;
  }
  
  .hacek {
   color: #cf1d24;
   font-size: 25px;
   font-weight: plain;
   vertical-align: -40%;
  }
  
  .mathematicapointerwrapper {
   border: 0px solid #DDDDDD;
   margin: 15px 15px 15px 40px;
   padding: 0px;
   max-width: 500px;
  }
  
  .mathematicapointertop {
   border: 1px solid #DDDDDD;
   background-color: #F2F2F2;
   padding: 0px;
   max-width: 500px;
   height: 4px;
  }
  
  .mathematicapointerdiv {
   background: url('./DocumentationFiles/mathematicabook.png') no-repeat  left center;
   border: 1px solid #DDDDDD;
   background-color: #FFFFFF;
   margin: 0px;
   padding: 15px 9px 9px 89px;
   max-width: 500px;
   min-height: 67px;
  }
  
  p.mathematicapointer {
   padding: 0px;
   margin: 0px;
   font-size: 12px;
  }
  
  .infoboxwrapper {
   border: 0px solid #DDDDDD;
   -webkit-box-shadow: 3px 3px 3px #DDDDDD;
   margin: 15px 15px 15px 40px;
   padding: 0px;
   max-width: 500px;
  }
  
  .infoboxtop {
   background: url('./DocumentationFiles/infotick.png') no-repeat left center;
   border: 1px solid #DDDDDD;
   background-color: #F2F2F2;
   padding: 0px;
   max-width: 500px;
   height: 37px;
  }
  
  div.infobox {
   border: 1px solid #DDDDDD;
   background-color: #FFFFFF;
   margin: 0px;
   padding: 15px;
   max-width: 500px;
  }
  
  p.infobox {
   padding: 0px;
   margin: 0px;
   font-size: 12px;
  }
  
  h2.legal {
   font-family: arial;
   font-size: 14;
   color: #cf1d24;
   margin: 15px 15px 15px 20px;
   font-weight: bold;
  }
  
  h3.legal {
   background: url('./DocumentationFiles/dingbat3.png') no-repeat 0 0;
   font-family: arial;
   font-size: 12;
   color: #808080;
   margin-left: 38px;
   padding-left: 12px;
   font-weight: bold;
  }
  
  ul.legal {
   font-size: 10px;
   font-family: arial;
   color: #555555;
   margin-left: 28px;
  }
  
  ul.legal li {
   margin-left: 0px;
   margin-top: 4px;
   margin-bottom: 2px;
   padding-left: 0px;
  }

  ul.imgbullets li {
   margin-left: -40px;
   margin-top: 4px;
   margin-bottom: 10px;
   padding-left: 30px;
   list-style-type: none;
  }
  
  p.legallarge {
   font-size: 12px;
   margin-left: 38px;
  }
  
  p.legalsmall {
   font-size: 11px;
   margin-left: 38px;
   padding-left: 12px;
  }
  
  .legalend {
   height: 10px;
  }
  
  .variablename {
   font-family: Courier New, Courier;
  }
  
  .dialogelement {
   font-weight: bold;
  }
  
  .menuitem {
   font-weight: bold;
  }
  
  .mr {
   font-family: Courier New, Courier;
  }
  
  .ttable {
	border-collapse: collapse;
    width: 100%;
	}

	.ttable td, th {
    border: 1px solid #dddddd;
    text-align: left;
	padding: 8px;
	}

	.ttable tr:nth-child(even) {
		background-color: #dddddd;
	}
  
   </style>
   </head>
   <body>
   
  <div class=\"headerlinkdiv\">
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/examples.png) no-repeat 0 0;
   padding-left: 24px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/examples_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/examples.png) no-repeat 0 0';
   \"><a href=\"https://www.wolfram.com/system-modeler/examples/\" class=\"headera\">More Examples</a></span>
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/video.png) no-repeat 0 0;
   padding-left: 29px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/video_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/video.png) no-repeat 0 0';
   \"><a href=\"http://www.wolfram.com/system-modeler/resources/get-started/\" class=\"headera\">Introductory Videos</a></span>
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/docs.png) no-repeat 0 0;
   padding-left: 20px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/docs_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/docs.png) no-repeat 0 0';
   \"><a href=\"http://reference.wolfram.com/system-modeler\" class=\"headera\">Documentation</a></span>
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/contact.png) no-repeat 0 0;
   padding-left: 24px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/contact_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/contact.png) no-repeat 0 0';
   \"><a href=\"http://www.wolfram.com/system-modeler/contact-us/\" class=\"headera\">Contact Us</a></span>
  </div> <div class=\"header\">&nbsp;</div> <h1>BasePendulumModel</h1><a id=\"headerTag_Introduction\" class=\"target\">&nbsp;</a>
    <h2>Introduction</h2>
    <p class=\"\">
This model is a partial model containing the dynamics of the inverted pendelum.
</p><h2 class=\"legal\"> <a href=\"#\" onclick=\"
   if(document.getElementById('legalSection').style.display == 'none'){
   document.getElementById('legalSection').style.display = 'block';
   document.getElementById('showlegalSection').style.display = 'none';
   document.getElementById('hidelegalSection').style.display = 'inline';
   } else {
   document.getElementById('legalSection').style.display = 'none';
   document.getElementById('showlegalSection').style.display = 'inline';
   document.getElementById('hidelegalSection').style.display = 'none';
   };
   return false;\" style=\"text-decoration: inherit; color: inherit\"><img src=\"./DocumentationFiles/showhide2.png\" alt=\"Show\" id=\"showlegalSection\" style=\"display: inline; vertical-align: text-bottom;\" /><img src=\"./DocumentationFiles/showhide.png\" alt=\"Hide\" id=\"hidelegalSection\" style=\"display: none; vertical-align: text-bottom;\" />Terms and Conditions of Use</a> </h2>
    <span id=\"legalSection\" style=\"display: none;\"><p class=\"legallarge\">
This domain example is an informational resource made freely available by Wolfram Research.
</p><h3 class=\"legal\">Use of This Example</h3>
    <span id=\"legalSection\" style=\"display: block;\"><ul class=\"legal\"><li>You may not use this example for any purpose that is unlawful or dangerous.</li><li>You assume total responsibility and risk for your use of this example.</li><li>You may not present this example with any alteration to its trade dress, trademarks, or design.</li></ul></span><h3 class=\"legal\">License</h3>
    <span id=\"legalSection\" style=\"display: block;\"><p class=\"legalsmall\">
All content in this bundle is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License. By accessing the content or using it in any way, you accept and agree to be bound by the terms of this license. If you do not agree to these Terms of Use, you may not use this content. Wolfram Research reserves the right to change, modify, add to, or remove portions of these Terms of Use at any time without notice. Please refer back to <a href=\"http://www.wolfram.com\">www.wolfram.com</a> for the latest Terms of Use.
</p><p class=\"legalsmall\">
A summary of the licensing terms can be found at:<br>
        <a href=\"http://creativecommons.org/licenses/by-nc-sa/3.0\">http://creativecommons.org/licenses/by-nc-sa/3.0</a>
</p><p class=\"legalsmall\">
The full legal code can be found at:<br>
        <a href=\"http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode\">http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode</a>
</p></span></span><div class=\"legalend\">&nbsp;</div>
   </body>
   <!--WSMINSERTIONTAGEND InvertedPendulum.Components.BasePendulumModel --></html>", revisions = ""), __Wolfram(PlotSet(plots = {Plot(name = "Controller Inputs", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = s, legend = "Cart Position"), Curve(x = time, y = v, legend = "Cart Velocity"), Curve(x = time, y = phi, legend = "Pendulum Angle"), Curve(x = time, y = w, legend = "Pendulum Anglular Velocity")})}), Plot(name = "Controller Output", subPlots = {SubPlot(curves = {Curve(x = time, y = y[1], legend = "Controller Output")}, label = Label(x = auto, y = ""))}), Plot(name = "Cart States", subPlots = {SubPlot(curves = {Curve(x = time, y = s, legend = "Cart Position"), Curve(x = time, y = v, legend = "Cart Velocity"), Curve(x = time, y = a, legend = "Cart Acceleration")})}), Plot(name = "Pendulum States", subPlots = {SubPlot(curves = {Curve(x = time, y = phi, legend = "Pendulum Angle"), Curve(x = time, y = w, legend = "Pendulum Anglular Velocity"), Curve(x = time, y = z, legend = "Pendulum Anglular Acceleration")})}), Plot(name = "Pendulum X-Y Position", subPlots = {SubPlot(curves = {Curve(x = pendelum_pos[1], y = pendelum_pos[2])}, label = Label(x = "X", y = "Y"))}), Plot(name = "Disturbance Forces", subPlots = {SubPlot(curves = {Curve(x = time, y = cartDisturbance[1], legend = "x-disturbance on cart"), Curve(x = time, y = cartDisturbance[2], legend = "y-disturbance on cart"), Curve(x = time, y = cartDisturbance[3], legend = "Torque-disturbance on cart")}, label = Label(x = auto, y = "[N]/[N.m]")), SubPlot(curves = {Curve(x = time, y = pendelumDisturbance[1], legend = "x-disturbance on pendulum"), Curve(x = time, y = pendelumDisturbance[2], legend = "y-disturbance on pendulum"), Curve(x = time, y = pendelumDisturbance[3], legend = "Torque-disturbance on pendulum")}, range = Range(xmin = 0, xmax = 20, ymin = auto, ymax = auto), label = Label(x = auto, y = "[N]/[N.m]"))}), Plot(name = "Corrective Force", subPlots = {SubPlot(curves = {Curve(x = time, y = controllerForce.force[1], legend = "Corrective force on cart, along x-axis")}, label = Label(x = auto, y = "[N]"))})})), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end BasePendulumModel;
    annotation(Documentation(info = "<html><!--WSMINSERTIONTAGSTART InvertedPendulum.Components -->
   <head>
   <style type=\"text/css\">
   
  body {
   padding: 0px;
   margin: 0px;
  }
  
  a {
   color: #cf1d24;
  }
  
  a.target {
   padding-top: 40px;
   margin-top: -40px;
  }
  
  p {
   font-family: arial;
   font-size: 13;
   margin: 9px 40px 5px 40px;
   padding-bottom: 0px;
   color: #555555;
   max-width: 800px;
  }
  
  h1 {
   font-size: 30;
   color: #cf1d24;
   font-weight: bold;
   margin-left: 20px;
   margin-top: 32px;
   margin-bottom: 15px;
   margin-right: 20px;
   padding-top: 0px;
  }
  
  h2 {
   font-size: 20;
   color: #cf1d24;
   font-weight: bold;
   margin-left: 20px;
   margin-right: 20px;
   margin-top: 5px;
   margin-bottom: 9px;
  }
  
  h3 {
   background: url('DocumentationFiles/dingbat3.png') no-repeat 0 0;
   font-size: 14px;
   font-family: helvetica;
   color: #4B4B4B;
   font-weight: bold;
   padding-left: 18px;
   margin-left: 20px;
   margin-right: 20px;
   margin-top: 12px;
   margin-bottom: 8px;
  }
  
  ul {
   font-family: arial;
   font-size: 13;
   margin: 9px 40px 5px 40px;
   padding-bottom: 0px;
   color: #555555;
   max-width: 800px;
   list-style-type: square;
  }
  
  li {
   margin-left: 0px;
   margin-top: 4px;
   margin-bottom: 2px;
   padding-left: 0px;
  }

  div.header {
   background: url('DocumentationFiles/WSMLogo.png') no-repeat 0 0;
   height: 96px;
   margin-top: 35px;
   background-color: #871613;
  }
  
  .headerspan {
   font-family: arial;
   text-decoration: none;
   font-size: 12px;
   font-weight: bold;
   display: inline-block;
   height: 35px;
   color: gray;
   padding: 0px;
   margin: 0px;
   margin-left: 20px;
  }
  
  .headera {
   font-family: arial;
   text-decoration: none;
   font-size: 12px;
   font-weight: bold;
   padding: 0px;
   color: inherit;
   vertical-align: middle;
   margin: 0px;
  }
  
  .headerlinkdiv {
   background: black;
   padding: 0px;
   height: 35px;
   margin: 0px;
   position: fixed; top: 0px; left: 0px; width: 100%;
  }
  
  .contenttable {
   -webkit-box-shadow: 3px 3px 3px #DDDDDD;
   border-top: 3px solid #cf1d24;
   background: #f9f9f9;
   max-width: 500px;
   margin: 15px 15px 0px 20px;
   padding: 6px 10px 3px 10px;
  }
  
  .contenttableheader {
   color: #a4a4a4;
   font-size: 14px;
   font-family: arial;
  }
  
  .contenttabletable {
   border: 0px solid #FFFFFF;
   padding: 0px;
   padding-left: 20px;
  }
  
  .contenttable tr td {
   padding: 3px;
   min-width: 200px;
  }
  
  .contenttable tr td a {
   color: #555555;
   text-decoration: none;
   font-size: 13px;
   font-family: arial;
  }
  
  .hacek {
   color: #cf1d24;
   font-size: 25px;
   font-weight: plain;
   vertical-align: -40%;
  }
  
  .mathematicapointerwrapper {
   border: 0px solid #DDDDDD;
   margin: 15px 15px 15px 40px;
   padding: 0px;
   max-width: 500px;
  }
  
  .mathematicapointertop {
   border: 1px solid #DDDDDD;
   background-color: #F2F2F2;
   padding: 0px;
   max-width: 500px;
   height: 4px;
  }
  
  .mathematicapointerdiv {
   background: url('./DocumentationFiles/mathematicabook.png') no-repeat  left center;
   border: 1px solid #DDDDDD;
   background-color: #FFFFFF;
   margin: 0px;
   padding: 15px 9px 9px 89px;
   max-width: 500px;
   min-height: 67px;
  }
  
  p.mathematicapointer {
   padding: 0px;
   margin: 0px;
   font-size: 12px;
  }
  
  .infoboxwrapper {
   border: 0px solid #DDDDDD;
   -webkit-box-shadow: 3px 3px 3px #DDDDDD;
   margin: 15px 15px 15px 40px;
   padding: 0px;
   max-width: 500px;
  }
  
  .infoboxtop {
   background: url('./DocumentationFiles/infotick.png') no-repeat left center;
   border: 1px solid #DDDDDD;
   background-color: #F2F2F2;
   padding: 0px;
   max-width: 500px;
   height: 37px;
  }
  
  div.infobox {
   border: 1px solid #DDDDDD;
   background-color: #FFFFFF;
   margin: 0px;
   padding: 15px;
   max-width: 500px;
  }
  
  p.infobox {
   padding: 0px;
   margin: 0px;
   font-size: 12px;
  }
  
  h2.legal {
   font-family: arial;
   font-size: 14;
   color: #cf1d24;
   margin: 15px 15px 15px 20px;
   font-weight: bold;
  }
  
  h3.legal {
   background: url('./DocumentationFiles/dingbat3.png') no-repeat 0 0;
   font-family: arial;
   font-size: 12;
   color: #808080;
   margin-left: 38px;
   padding-left: 12px;
   font-weight: bold;
  }
  
  ul.legal {
   font-size: 10px;
   font-family: arial;
   color: #555555;
   margin-left: 28px;
  }
  
  ul.legal li {
   margin-left: 0px;
   margin-top: 4px;
   margin-bottom: 2px;
   padding-left: 0px;
  }

  ul.imgbullets li {
   margin-left: -40px;
   margin-top: 4px;
   margin-bottom: 10px;
   padding-left: 30px;
   list-style-type: none;
  }
  
  p.legallarge {
   font-size: 12px;
   margin-left: 38px;
  }
  
  p.legalsmall {
   font-size: 11px;
   margin-left: 38px;
   padding-left: 12px;
  }
  
  .legalend {
   height: 10px;
  }
  
  .variablename {
   font-family: Courier New, Courier;
  }
  
  .dialogelement {
   font-weight: bold;
  }
  
  .menuitem {
   font-weight: bold;
  }
  
  .mr {
   font-family: Courier New, Courier;
  }
  
  .ttable {
	border-collapse: collapse;
    width: 100%;
	}

	.ttable td, th {
    border: 1px solid #dddddd;
    text-align: left;
	padding: 8px;
	}

	.ttable tr:nth-child(even) {
		background-color: #dddddd;
	}
  
   </style>
   </head>
   <body>
   
  <div class=\"headerlinkdiv\">
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/examples.png) no-repeat 0 0;
   padding-left: 24px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/examples_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/examples.png) no-repeat 0 0';
   \"><a href=\"https://www.wolfram.com/system-modeler/examples/\" class=\"headera\">More Examples</a></span>
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/video.png) no-repeat 0 0;
   padding-left: 29px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/video_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/video.png) no-repeat 0 0';
   \"><a href=\"http://www.wolfram.com/system-modeler/resources/get-started/\" class=\"headera\">Introductory Videos</a></span>
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/docs.png) no-repeat 0 0;
   padding-left: 20px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/docs_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/docs.png) no-repeat 0 0';
   \"><a href=\"http://reference.wolfram.com/system-modeler\" class=\"headera\">Documentation</a></span>
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/contact.png) no-repeat 0 0;
   padding-left: 24px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/contact_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/contact.png) no-repeat 0 0';
   \"><a href=\"http://www.wolfram.com/system-modeler/contact-us/\" class=\"headera\">Contact Us</a></span>
  </div> <div class=\"header\">&nbsp;</div> <h1>LQR-controlled inverted pendulum system</h1><a id=\"headerTag_PackageOverview\" class=\"target\">&nbsp;</a>
    <h2>Package Overview</h2>
    <p class=\"\">
This is a package containing submodels.
</p><ul><li><a href=\"modelica://InvertedPendulum.Components.BasePendulumModel#diagram\">BasePendulumModel</a>: Partial inverted pendelum model (<a href=\"modelica://InvertedPendulum.Components.BasePendulumModel\">show documentation</a>)</li></ul><h2 class=\"legal\"> <a href=\"#\" onclick=\"
   if(document.getElementById('legalSection').style.display == 'none'){
   document.getElementById('legalSection').style.display = 'block';
   document.getElementById('showlegalSection').style.display = 'none';
   document.getElementById('hidelegalSection').style.display = 'inline';
   } else {
   document.getElementById('legalSection').style.display = 'none';
   document.getElementById('showlegalSection').style.display = 'inline';
   document.getElementById('hidelegalSection').style.display = 'none';
   };
   return false;\" style=\"text-decoration: inherit; color: inherit\"><img src=\"./DocumentationFiles/showhide2.png\" alt=\"Show\" id=\"showlegalSection\" style=\"display: inline; vertical-align: text-bottom;\" /><img src=\"./DocumentationFiles/showhide.png\" alt=\"Hide\" id=\"hidelegalSection\" style=\"display: none; vertical-align: text-bottom;\" />Terms and Conditions of Use</a> </h2>
    <span id=\"legalSection\" style=\"display: none;\"><p class=\"legallarge\">
This domain example is an informational resource made freely available by Wolfram Research.
</p><h3 class=\"legal\">Use of This Example</h3>
    <span id=\"legalSection\" style=\"display: block;\"><ul class=\"legal\"><li>You may not use this example for any purpose that is unlawful or dangerous.</li><li>You assume total responsibility and risk for your use of this example.</li><li>You may not present this example with any alteration to its trade dress, trademarks, or design.</li></ul></span><h3 class=\"legal\">License</h3>
    <span id=\"legalSection\" style=\"display: block;\"><p class=\"legalsmall\">
All content in this bundle is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License. By accessing the content or using it in any way, you accept and agree to be bound by the terms of this license. If you do not agree to these Terms of Use, you may not use this content. Wolfram Research reserves the right to change, modify, add to, or remove portions of these Terms of Use at any time without notice. Please refer back to <a href=\"http://www.wolfram.com\">www.wolfram.com</a> for the latest Terms of Use.
</p><p class=\"legalsmall\">
A summary of the licensing terms can be found at:<br>
        <a href=\"http://creativecommons.org/licenses/by-nc-sa/3.0\">http://creativecommons.org/licenses/by-nc-sa/3.0</a>
</p><p class=\"legalsmall\">
The full legal code can be found at:<br>
        <a href=\"http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode\">http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode</a>
</p></span></span><div class=\"legalend\">&nbsp;</div>
   </body>
   <!--WSMINSERTIONTAGEND InvertedPendulum.Components --></html>", revisions = ""), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end Components;

  model InvertedPendulumPulse "A LQR-controlled inverted pendulum system"
    extends Components.BasePendulumModel;
    parameter Modelica.Units.SI.Force cartDisturbanceForce = 0.1 "Ampliude of disturbance force applied to cart." annotation(Dialog(group = "Disturbance Properties"));
    parameter Modelica.Units.SI.Force pendulumDisturbanceForce = 1.5 "Ampliude of disturbance force applied to the top of the pendulum." annotation(Dialog(group = "Disturbance Properties"));
    parameter Modelica.Units.SI.Time cartDisturbancePeriod = 4 "Period of the disturbance pulse on the cart." annotation(Dialog(group = "Disturbance Properties"));
    parameter Modelica.Units.SI.Time pendulumDisturbancePeriod = 1 "Period of the disturbance pulse on the top of the pendulum." annotation(Dialog(group = "Disturbance Properties"));
    Modelica.Blocks.Sources.Pulse disturbanceCartPulse(amplitude = cartDisturbanceForce, period = cartDisturbancePeriod, nperiod = 5, startTime = 4) "Magnitude of disturbance force applied on the cart." annotation(Placement(visible = true, transformation(origin = {60, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Pulse disturbancePendulumPulse(amplitude = pendulumDisturbanceForce, width = 100, period = pendulumDisturbancePeriod, nperiod = 1, startTime = 6) "Magnitude of disturbance force applied on the pendulum." annotation(Placement(visible = true, transformation(origin = {80, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    disturbancePendulum.force[2:3] = {0, 0};
    disturbanceCart.force[2:3] = {0, 0};
    controllerForce.force[2:3] = {0, 0};
    connect(disturbanceCartPulse.y, disturbanceCart.force[1]) annotation(Line(visible = true, origin = {25.5, -50}, points = {{23.5, 0}, {-23.5, 0}}, color = {1, 37, 163}));
    connect(disturbancePendulumPulse.y, disturbancePendulum.force[1]) annotation(Line(visible = true, origin = {35.5, -70}, points = {{33.5, 0}, {-33.5, 0}}, color = {1, 37, 163}));
    annotation(Documentation(info = "<html><!--WSMINSERTIONTAGSTART InvertedPendulum.InvertedPendulumPulse -->
   <head>
   <style type=\"text/css\">
   
  body {
   padding: 0px;
   margin: 0px;
  }
  
  a {
   color: #cf1d24;
  }
  
  a.target {
   padding-top: 40px;
   margin-top: -40px;
  }
  
  p {
   font-family: arial;
   font-size: 13;
   margin: 9px 40px 5px 40px;
   padding-bottom: 0px;
   color: #555555;
   max-width: 800px;
  }
  
  h1 {
   font-size: 30;
   color: #cf1d24;
   font-weight: bold;
   margin-left: 20px;
   margin-top: 32px;
   margin-bottom: 15px;
   margin-right: 20px;
   padding-top: 0px;
  }
  
  h2 {
   font-size: 20;
   color: #cf1d24;
   font-weight: bold;
   margin-left: 20px;
   margin-right: 20px;
   margin-top: 5px;
   margin-bottom: 9px;
  }
  
  h3 {
   background: url('DocumentationFiles/dingbat3.png') no-repeat 0 0;
   font-size: 14px;
   font-family: helvetica;
   color: #4B4B4B;
   font-weight: bold;
   padding-left: 18px;
   margin-left: 20px;
   margin-right: 20px;
   margin-top: 12px;
   margin-bottom: 8px;
  }
  
  ul {
   font-family: arial;
   font-size: 13;
   margin: 9px 40px 5px 40px;
   padding-bottom: 0px;
   color: #555555;
   max-width: 800px;
   list-style-type: square;
  }
  
  li {
   margin-left: 0px;
   margin-top: 4px;
   margin-bottom: 2px;
   padding-left: 0px;
  }

  div.header {
   background: url('DocumentationFiles/WSMLogo.png') no-repeat 0 0;
   height: 96px;
   margin-top: 35px;
   background-color: #871613;
  }
  
  .headerspan {
   font-family: arial;
   text-decoration: none;
   font-size: 12px;
   font-weight: bold;
   display: inline-block;
   height: 35px;
   color: gray;
   padding: 0px;
   margin: 0px;
   margin-left: 20px;
  }
  
  .headera {
   font-family: arial;
   text-decoration: none;
   font-size: 12px;
   font-weight: bold;
   padding: 0px;
   color: inherit;
   vertical-align: middle;
   margin: 0px;
  }
  
  .headerlinkdiv {
   background: black;
   padding: 0px;
   height: 35px;
   margin: 0px;
   position: fixed; top: 0px; left: 0px; width: 100%;
  }
  
  .contenttable {
   -webkit-box-shadow: 3px 3px 3px #DDDDDD;
   border-top: 3px solid #cf1d24;
   background: #f9f9f9;
   max-width: 500px;
   margin: 15px 15px 0px 20px;
   padding: 6px 10px 3px 10px;
  }
  
  .contenttableheader {
   color: #a4a4a4;
   font-size: 14px;
   font-family: arial;
  }
  
  .contenttabletable {
   border: 0px solid #FFFFFF;
   padding: 0px;
   padding-left: 20px;
  }
  
  .contenttable tr td {
   padding: 3px;
   min-width: 200px;
  }
  
  .contenttable tr td a {
   color: #555555;
   text-decoration: none;
   font-size: 13px;
   font-family: arial;
  }
  
  .hacek {
   color: #cf1d24;
   font-size: 25px;
   font-weight: plain;
   vertical-align: -40%;
  }
  
  .mathematicapointerwrapper {
   border: 0px solid #DDDDDD;
   margin: 15px 15px 15px 40px;
   padding: 0px;
   max-width: 500px;
  }
  
  .mathematicapointertop {
   border: 1px solid #DDDDDD;
   background-color: #F2F2F2;
   padding: 0px;
   max-width: 500px;
   height: 4px;
  }
  
  .mathematicapointerdiv {
   background: url('./DocumentationFiles/mathematicabook.png') no-repeat  left center;
   border: 1px solid #DDDDDD;
   background-color: #FFFFFF;
   margin: 0px;
   padding: 15px 9px 9px 89px;
   max-width: 500px;
   min-height: 67px;
  }
  
  p.mathematicapointer {
   padding: 0px;
   margin: 0px;
   font-size: 12px;
  }
  
  .infoboxwrapper {
   border: 0px solid #DDDDDD;
   -webkit-box-shadow: 3px 3px 3px #DDDDDD;
   margin: 15px 15px 15px 40px;
   padding: 0px;
   max-width: 500px;
  }
  
  .infoboxtop {
   background: url('./DocumentationFiles/infotick.png') no-repeat left center;
   border: 1px solid #DDDDDD;
   background-color: #F2F2F2;
   padding: 0px;
   max-width: 500px;
   height: 37px;
  }
  
  div.infobox {
   border: 1px solid #DDDDDD;
   background-color: #FFFFFF;
   margin: 0px;
   padding: 15px;
   max-width: 500px;
  }
  
  p.infobox {
   padding: 0px;
   margin: 0px;
   font-size: 12px;
  }
  
  h2.legal {
   font-family: arial;
   font-size: 14;
   color: #cf1d24;
   margin: 15px 15px 15px 20px;
   font-weight: bold;
  }
  
  h3.legal {
   background: url('./DocumentationFiles/dingbat3.png') no-repeat 0 0;
   font-family: arial;
   font-size: 12;
   color: #808080;
   margin-left: 38px;
   padding-left: 12px;
   font-weight: bold;
  }
  
  ul.legal {
   font-size: 10px;
   font-family: arial;
   color: #555555;
   margin-left: 28px;
  }
  
  ul.legal li {
   margin-left: 0px;
   margin-top: 4px;
   margin-bottom: 2px;
   padding-left: 0px;
  }

  ul.imgbullets li {
   margin-left: -40px;
   margin-top: 4px;
   margin-bottom: 10px;
   padding-left: 30px;
   list-style-type: none;
  }
  
  p.legallarge {
   font-size: 12px;
   margin-left: 38px;
  }
  
  p.legalsmall {
   font-size: 11px;
   margin-left: 38px;
   padding-left: 12px;
  }
  
  .legalend {
   height: 10px;
  }
  
  .variablename {
   font-family: Courier New, Courier;
  }
  
  .dialogelement {
   font-weight: bold;
  }
  
  .menuitem {
   font-weight: bold;
  }
  
  .mr {
   font-family: Courier New, Courier;
  }
  
  .ttable {
	border-collapse: collapse;
    width: 100%;
	}

	.ttable td, th {
    border: 1px solid #dddddd;
    text-align: left;
	padding: 8px;
	}

	.ttable tr:nth-child(even) {
		background-color: #dddddd;
	}
  
   </style>
   </head>
   <body>
   
  <div class=\"headerlinkdiv\">
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/examples.png) no-repeat 0 0;
   padding-left: 24px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/examples_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/examples.png) no-repeat 0 0';
   \"><a href=\"https://www.wolfram.com/system-modeler/examples/\" class=\"headera\">More Examples</a></span>
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/video.png) no-repeat 0 0;
   padding-left: 29px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/video_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/video.png) no-repeat 0 0';
   \"><a href=\"http://www.wolfram.com/system-modeler/resources/get-started/\" class=\"headera\">Introductory Videos</a></span>
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/docs.png) no-repeat 0 0;
   padding-left: 20px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/docs_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/docs.png) no-repeat 0 0';
   \"><a href=\"http://reference.wolfram.com/system-modeler\" class=\"headera\">Documentation</a></span>
  
   <span class=\"headerspan\" style=\"background: url(DocumentationFiles/contact.png) no-repeat 0 0;
   padding-left: 24px;
   \"
   onmouseover=\"
   	this.style.color = 'white';
   	this.style.background = 'url(DocumentationFiles/contact_active.png) no-repeat 0 0';
   \"
   onmouseout=\"
   	this.style.color = 'gray';
   	this.style.background = 'url(DocumentationFiles/contact.png) no-repeat 0 0';
   \"><a href=\"http://www.wolfram.com/system-modeler/contact-us/\" class=\"headera\">Contact Us</a></span>
  </div> <div class=\"header\">&nbsp;</div> <h1>LQR-controlled inverted pendulum system</h1><div class=\"contenttable\">
   <span class=\"contenttableheader\">CONTENTS:</span><br/>
   <table class=\"contenttabletable\"><tr><td><a href=\"#\" onClick=\"document.getElementById('headerTag_Introduction').scrollIntoView(); return false;\"><span class=\"hacek\">&#711;</span> Introduction</a></td><td><a href=\"#\" onClick=\"document.getElementById('headerTag_Model').scrollIntoView(); return false;\"><span class=\"hacek\">&#711;</span> Model</a></td></tr><tr><td><a href=\"#\" onClick=\"document.getElementById('headerTag_ControlSystem').scrollIntoView(); return false;\"><span class=\"hacek\">&#711;</span> Control System</a></td><td><a href=\"#\" onClick=\"document.getElementById('headerTag_Simulation').scrollIntoView(); return false;\"><span class=\"hacek\">&#711;</span> Simulation</a></td></tr></table></div><a id=\"headerTag_Introduction\" class=\"target\">&nbsp;</a>
    <h2>Introduction</h2>
    <p class=\"\">
This model studies how a  linear-quadratic regulator (LQR) can be used to stabilize an inverted pendulum.
</p><p class=\"\">
The model requires the free <a href=\"https://www.wolfram.com/system-modeler/libraries/planar-mechanics/\">PlanarMechanics</a> library, availiable from the SystemModeler Library Store.
</p><a id=\"headerTag_Model\" class=\"target\">&nbsp;</a>
    <h2>Model</h2>
    <p class=\"\">
The inverted pendulum model consists of a pendulum and a cart, with the axis of rotation of the pendulum being located at the center of the cart. The pendulum is initialized with its center of mass above the rotation axis. When located directly above the cart, the pendulum will be in steady state and will stay there until disturbed.
</p><p class=\"\">
While a pendulum that hangs straight down will be in a stable position, an inverted pendulum is unstable. That means that any small disturbance will cause the pendulum to tip over and never return to its original position.
</p><p class=\"\">
<img src=\"modelica://InvertedPendulum/DocumentationFiles/pendulum.png\" alt=\"pendulum\"/>
</p><a id=\"headerTag_ControlSystem\" class=\"target\">&nbsp;</a>
    <h2>Control System</h2>
    <p class=\"\">
A control system can be used to stabilize the unstable pendulum. Here, a linear-quadratic regulator is used. First, the regulator measures the states of the system, namely the cart position and velocity, the pendulum angle and the angular velocity. The regulator then calculates a force that should be applied on the cart in order for all the states to become zero. In other words, the regulator aims to have the pendulum pointing straight up (defined here as 0 degrees) and the cart return to the origin.
</p><p class=\"\">
<img src=\"modelica://InvertedPendulum/DocumentationFiles/controlSystem.png\" alt=\"controlSystem\"/>
</p><a id=\"headerTag_Simulation\" class=\"target\">&nbsp;</a>
    <h2>Simulation</h2>
    <p class=\"\">
By simulating the model, you can try different values of the parameters and see how the control system responds.
</p><p class=\"\">
To simulate the model and see the generated 3D animation, follow the steps below:
</p><ul><li>Click the <span class=\"dialogelement\">Simulate</span> button <img src=\"modelica://InvertedPendulum/DocumentationFiles/simulate.png\" alt=\"simulate\"/>.</li><li>When the simulation is finished, click the <span class=\"dialogelement\">Animate</span> button <img src=\"modelica://InvertedPendulum/DocumentationFiles/animate.png\" alt=\"animate\"/> in Simulation Center.</li><li>Use your mouse or trackpad to drag the animation into a good angle and zoom in with your scroll wheel or by using the trackpad. Then click the <span class=\"dialogelement\">Play</span> button <img src=\"modelica://InvertedPendulum/DocumentationFiles/simPlay.png\" alt=\"simPlay\"/> to play the animation.</li></ul><p class=\"\">
You can change the magnitude of the force that is applied to the pendulum by changing the <span class=\"variablename\">pendulumDisturbanceForce</span> parameter.
</p><h2 class=\"legal\"> <a href=\"#\" onclick=\"
   if(document.getElementById('legalSection').style.display == 'none'){
   document.getElementById('legalSection').style.display = 'block';
   document.getElementById('showlegalSection').style.display = 'none';
   document.getElementById('hidelegalSection').style.display = 'inline';
   } else {
   document.getElementById('legalSection').style.display = 'none';
   document.getElementById('showlegalSection').style.display = 'inline';
   document.getElementById('hidelegalSection').style.display = 'none';
   };
   return false;\" style=\"text-decoration: inherit; color: inherit\"><img src=\"./DocumentationFiles/showhide2.png\" alt=\"Show\" id=\"showlegalSection\" style=\"display: inline; vertical-align: text-bottom;\" /><img src=\"./DocumentationFiles/showhide.png\" alt=\"Hide\" id=\"hidelegalSection\" style=\"display: none; vertical-align: text-bottom;\" />Terms and Conditions of Use</a> </h2>
    <span id=\"legalSection\" style=\"display: none;\"><p class=\"legallarge\">
This domain example is an informational resource made freely available by Wolfram Research.
</p><h3 class=\"legal\">Use of This Example</h3>
    <span id=\"legalSection\" style=\"display: block;\"><ul class=\"legal\"><li>You may not use this example for any purpose that is unlawful or dangerous.</li><li>You assume total responsibility and risk for your use of this example.</li><li>You may not present this example with any alteration to its trade dress, trademarks, or design.</li></ul></span><h3 class=\"legal\">License</h3>
    <span id=\"legalSection\" style=\"display: block;\"><p class=\"legalsmall\">
All content in this bundle is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License. By accessing the content or using it in any way, you accept and agree to be bound by the terms of this license. If you do not agree to these Terms of Use, you may not use this content. Wolfram Research reserves the right to change, modify, add to, or remove portions of these Terms of Use at any time without notice. Please refer back to <a href=\"http://www.wolfram.com\">www.wolfram.com</a> for the latest Terms of Use.
</p><p class=\"legalsmall\">
A summary of the licensing terms can be found at:<br>
        <a href=\"http://creativecommons.org/licenses/by-nc-sa/3.0\">http://creativecommons.org/licenses/by-nc-sa/3.0</a>
</p><p class=\"legalsmall\">
The full legal code can be found at:<br>
        <a href=\"http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode\">http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode</a>
</p></span></span><div class=\"legalend\">&nbsp;</div>
   </body>
   <!--WSMINSERTIONTAGEND InvertedPendulum.InvertedPendulumPulse --></html>", revisions = ""), experiment(StopTime = 20, __Wolfram_SynchronizeWithRealTime = false), __Wolfram, Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end InvertedPendulumPulse;
end InvertedPendulum;