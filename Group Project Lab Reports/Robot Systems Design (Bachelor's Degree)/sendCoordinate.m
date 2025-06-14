X1 = First_red(2) +200;
Y1 = First_red(1) -320 ;
R1 = First_red(3) +90 ;

Xred = num2str(X1);
Yred = num2str(Y1);
Rred = num2str(R1);

X2 = First_yellow(2) +200;
Y2 = First_yellow(1) -320;
R2 = First_yellow(3) +90; 

Xyellow = num2str(X2);
Yyellow = num2str(Y2);
Ryellow = num2str(R2);

X3 = First_blue(2) +200;
Y3 = First_blue(1) -320;
R3 = First_blue(3) +90; 

Xblue = num2str(X3);
Yblue = num2str(Y3);
Rblue = num2str(R3);

X4 = First_green(2) +200;
Y4 = First_green(1) -320;
R4 = First_green(3) +90; 

Xgreen = num2str(X4);
Ygreen = num2str(Y4);
Rgreen = num2str(R4);

con=pnet('tcpconnect','127.0.0.1',1025);
pnet(con,'setreadtimeout',10);
pnet(con,'printf',Xred);
pnet(con,'readline')
pnet(con,'printf',Yred);
pnet(con,'readline')
pnet(con,'printf',Rred);
pnet(con,'readline')
pnet(con,'printf',Xyellow);
pnet(con,'readline')
pnet(con,'printf',Yyellow);
pnet(con,'readline')
pnet(con,'printf',Ryellow);
pnet(con,'readline')
pnet(con,'printf',Xblue);
pnet(con,'readline')
pnet(con,'printf',Yblue);
pnet(con,'readline')
pnet(con,'printf',Rblue);
pnet(con,'readline')
pnet(con,'printf',Xgreen);
pnet(con,'readline')
pnet(con,'printf',Ygreen);
pnet(con,'readline')
pnet(con,'printf',Rgreen);
pnet(con,'readline')

