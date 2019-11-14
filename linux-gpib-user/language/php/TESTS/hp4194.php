#!/usr/bin/php -q
<?
$u = ibfind("dev1");
print "numero d'unite = $u\n";
ibpad($u,23) or die("ibpad");
ibwrt($u,"RST");
ibwrt($u, "FNC2;SWT2;ATR2;ATT2;ZIR1;ZIT1;ITM2");
ibwrt($u, "NOA=4;NOP=41");
ibwrt($u, "START=100HZ;STOP=1MHZ");
ibwrt($u, "OSC=50mV");
ibrsp($u, &$status);
print "le status de rsp est $status\n";
ibwrt($u, "RQS2");
print "debut mesure....\n";
ibwrt($u, "SWTRG");
print "... mesure lancee\n";
ibwait($u,RQS);
print "... mesure finitte\n";
/*
ibwrt($u, "STB?");
$reponse=ibrd($u,&$tampon,30);
$v = 0+$tampon;
print "la reponse STB est $v\n";
*/

ibwrt($u,"A?");
$reponse=ibrd($u,&$tampon,10000);
eval ("\$A = array($tampon);");
ibwrt($u,"B?");
$reponse=ibrd($u,&$tampon,10000);
eval ("\$B = array($tampon);");
ibwrt($u,"X?");
$reponse=ibrd($u,&$tampon,10000);
eval ("\$X = array($tampon);");

$count = count($A);
for ($i=0; $i<$count; $i++) {
    print "$A[$i]\t$B[$i]\t$X[$i]\n";
}

?>
       
