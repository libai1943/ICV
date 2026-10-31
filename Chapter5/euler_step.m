function next=euler_step(state,input,step,wheelbase)
% First-order explicit forward Euler, used by every transcription and initializer.
next=state+step*vehicle_rhs(state,input,wheelbase);
end
