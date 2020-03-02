function [H,C,B,Cr,PE,KE,Pcom,Vcom,Pjt] = eom( robot )
%function [PE,Pcom,Vcom,Pjt] = eom( robot )

%     fprintf('\n==> Derive equations of motion given by:')
%     fprintf('\nH(q)qdd + C(q,qd)qd + G(q) = B(q)u')
%     tic
%     fprintf('\n 1/3: RNEA for C(q,qd)qd+G(q) and CRBA for H(q)...')
%    fprintf('\n 1/3: ')
    
    [H,C,B] = HandC( robot, robot.q, robot.qd );
    %[Ma,C] = HandC( robot, robot.q_sym, robot.qd_sym );
    %fprintf([num2str(toc,'%.4f'),' sec.'])

    tic
    fprintf('Extract C(q,qd)...')
    Cr = 1/2*jacobian(C,robot.qd(:));
    fprintf([num2str(toc,'%.4f'),' sec.'])
  
    %tic
    %fprintf('\n 3/8: Get G(q)...')
    %Gr = robot.g_sym*jacobian(C,robot.g_sym);
    %fprintf([num2str(toc,'%.4f'),' sec.'])

    %tic
    %fprintf('\n 3/7: Export H(q)...')
    %robot.H_fun = matlabFunction(Ma,'vars',robot.states(1:robot.NB));
    %fprintf([num2str(toc,'%.4f'),' sec.'])
    
    %tic
    %fprintf('\n 4/7: Export C(q,qd)qd...')
    %robot.C_fun = matlabFunction(C,'vars',{robot.states{1:2*robot.NB},robot.g_sym});
    %fprintf([num2str(toc,'%.4f'),' sec.'])

    tic
    fprintf('\nGet PE, KE, Pjt, Pcom, and Vcom...')
    ret = EnerMo( robot, robot.q, robot.qd );
    fprintf([num2str(toc,'%.4f'),' sec.'])
    
    %robot.KE_fun = matlabFunction(ret.KE,'vars',robot.states(1:2*robot.NB));
    %robot.PE_fun = matlabFunction(ret.PE,'vars',{robot.states{1:robot.NB},robot.g_sym});

    KE = ret.KE;    
    PE = ret.PE;
    Pcom = ret.cm;
    Vcom = ret.vcm;
    Pjt = ret.Pjt;
    
    %tic
    %fprintf('\n 6/7: Export Pcom(q)...')
    %robot.Pcom_fun = matlabFunction(ret.cm,'vars',robot.states(1:robot.NB));
    %fprintf([num2str(toc,'%.4f'),' sec.'])

    %tic
    %fprintf('\n 7/7: Export Vcom(q,qd)...')
    %robot.vcom_fun = matlabFunction(ret.vcm,'vars',robot.states(1:2*robot.NB));
    %fprintf([num2str(toc,'%.4f'),' sec.\n\n'])

    
end