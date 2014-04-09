function moveFocuser(posn)
{
while (F.IsMoving)
{
WScript.sleep(10)
//WScript.StdOut.Write("Sleep  ")

}
F.Move(posn)

return true
}

var F = new ActiveXObject("ASCOM.LYFocuser.Focuser");    // Change for your driver's ID
F.SetupDialog();                                    // Comment this out once you set COM port, etc.
F.Connected = true;
startPosition = F.Position 
position = startPosition
minorMove=10
majorMove=50
minorMoveCount=10
majorMoveCount=10
WScript.StdOut.WriteLine("Start of Focuser  = " +startPosition);
for (i=0; i<majorMoveCount;i++)
{

position=position+majorMove
moveFocuser(position)
WScript.StdOut.Write("Major Loop Count= "+i);
WScript.StdOut.WriteLine(" ---Major Loop position= "+position);
for (j=0; j<minorMoveCount;j++)
		{position=position+minorMove
		moveFocuser(position)
		WScript.StdOut.WriteLine("Minor Loop position= "+position);
		}
for (j=0; j<minorMoveCount;j++)
		{position=position-minorMove
		moveFocuser(position)
		WScript.StdOut.WriteLine("Minor Loop position= "+position);
		}
		
}
// NOW work backwards

for (i=0; i<majorMoveCount;i++)
{

position=position-majorMove
F.Move(position)
WScript.StdOut.Write("Major Loop Count= "+i);
WScript.StdOut.WriteLine(" ---Major Loop position= "+position);
for (j=0; j<minorMoveCount;j++)
		{position=position+minorMove
		moveFocuser(position)
		WScript.StdOut.WriteLine("Minor Loop position= "+position);
		}
for (j=0; j<minorMoveCount;j++)
		{position=position-minorMove
		moveFocuser(position)
		WScript.StdOut.WriteLine("Minor Loop position= "+position);
		}
		
}

//if (F.CanSetTracking && !F.Tracking)
  //  F.Tracking = true;
//WScript.StdOut.WriteLine("Slewing to 1 hour east of meridian...");
//F.SlewToCoordinates(F.SiderealTime + 1.0, 0);
//WScript.StdOut.WriteLine("... slew complete");
WScript.StdOut.Write("Press Enter to quit and release the driver ");
WScript.StdIn.ReadLine();