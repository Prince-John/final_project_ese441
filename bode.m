A = [0 1; 0 0];
B = [0;1];
C = [1 0]; 
D =0; 

B_ = [0; 1]; 
C_ = [0 0]; D_ = 2; 

sys_1 = ss(A, B, C, D); 
sys_2 = ss(A, B_, C_, D_); 

transfer_1 = tf(sys_1); 
transfer_2 = tf(sys_2); 

final_transfer = transfer_1/(1+transfer_1*transfer_2); 
%final_transfer = transfer_1*transfer_2; 
bode(final_transfer)