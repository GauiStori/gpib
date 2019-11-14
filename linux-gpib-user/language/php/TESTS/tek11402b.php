#!/usr/local/bin/php
<?php
{


	$ii = 0;
	$xx = "bbbbb";

	$dev = ibdev(0,5,0,T3s,1,0);
	print "Number of dev is $dev\n";

	$xx = "ID?";

	$ii = ibwrt($dev,$xx,strlen($xx));
	$zz = ibcnt();
	echo("ibwrt returns: $ii $zz\n");
	$err = iberr();
	
	if($err > 0) {
	  echo("iberr: $err\n");
	  echo(gpib_error_string($err));
	  echo("\n");
	}
	
	$ii = ibrd($dev,$xx,1000);
	$zz = ibcnt();
	echo("ibrd returns: $ii $zz\n");
	echo("($xx)\n");

	$err = iberr();
	if($err > 0) {
	  echo("iberr: $err\n");
	  
	  echo(gpib_error_string($err));
	  echo("\n");
	}

	
	$xx = "infile1.txt";

	$ii = ibwrtf($dev,$xx);
	$zz = ibcnt();
	echo("ibwrt returns: $ii $zz\n");
	$err = iberr();
	
	if($err > 0) {
	  echo("iberr: $err\n");
	  echo(gpib_error_string($err));
	  echo("\n");
	}
	
	$ii = ibrdf($dev,"outfile1.txt");
	$zz = ibcnt();
	echo("ibrd returns: $ii $zz\n");
	echo("($xx)\n");

	$err = iberr();
	if($err > 0) {
	  echo("iberr: $err\n");
	  
	  echo(gpib_error_string($err));
	  echo("\n");
	}

	
	$xx = "infile2.txt";

	$ii = ibwrtf($dev,$xx);
	$zz = ibcnt();
	echo("ibwrt returns: $ii $zz\n");
	$err = iberr();
	
	if($err > 0) {
	  echo("iberr: $err\n");
	  echo(gpib_error_string($err));
	  echo("\n");
	}
	
	$ii = ibrdf($dev,"outfile2.txt");
	$zz = ibcnt();
	echo("ibrd returns: $ii $zz\n");
	echo("($xx)\n");

	$err = iberr();
	if($err > 0) {
	  echo("iberr: $err\n");
	  
	  echo(gpib_error_string($err));
	  echo("\n");
	}

	echo("done.\n");

}
?>
