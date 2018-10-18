
% drawing



% A = importdata('XOR/Train_epoch_mse');
% 
% % every epoch dt
% dt = 100;
% plot(A(:,1)/dt,A(:,2),'r-o');
% 
% 
% title('XOR Neuro Network Training MSE','FontName','Times New Roman','FontSize',12);
% ylabel('MSE','FontName','Times New Roman','FontSize',12);
% show_dt = num2str(dt);
% xlabel(['Every ' show_dt ' epoch'],'FontName','Times New Roman','FontSize',12);

% figure(2)
A = importdata('Hands/Train_epoch_mse');

% every epoch dt
dt = 100;
plot(A(:,1)/dt,A(:,2),'b-*');

title('Neuro Network Training MSE','FontName','Times New Roman','FontSize',12);
ylabel('MSE','FontName','Times New Roman','FontSize',12);
show_dt = num2str(dt);
xlabel(['Every ' show_dt ' epoch'],'FontName','Times New Roman','FontSize',12);
  
