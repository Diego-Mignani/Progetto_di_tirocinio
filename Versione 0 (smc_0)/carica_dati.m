function conferma = carica_dati(gamma,lambda,theta,epsilon,ISE, IAE, ITAE, error)
 
persistent riga;
persistent colonna;

if isempty(riga) || riga>28|| isempty(colonna)
    riga = 1;
    colonna = 'M';
end

s1 = strcat('A', num2str(riga));
s2 = strcat(colonna,'1');
T = table(gamma, lambda, theta, epsilon, ISE, IAE, ITAE);
filename = 'dati.xlsx';
writetable(T,filename,'Sheet',1,'Range',s1)
writematrix(error, filename,'Sheet',1,'Range',s2)


riga = riga+2;
colonna = char(colonna+1);
conferma = 1;