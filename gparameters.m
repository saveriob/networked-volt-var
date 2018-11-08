function [G, X] = gparameters(mpc,c)

Y = makeYbus(mpc);
n = size(Y,1);
BUS_TYPE = 2;
pcc = find(mpc.bus(:,BUS_TYPE)==3);

% compute X as in [doi: 10.1109/TAC.2013.2270317]

YY = zeros(n+1);
YY(1:n,1:n) = Y;
YY(pcc,end) = 1;
YY(end, pcc) = 1;

XX = inv(YY);
X = imag(XX(c,c));

G = inv(X);