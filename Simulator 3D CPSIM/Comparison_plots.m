te = load("TRANSMITTER_ESTIMATE_ERROR.mat");
te_nod = load("TRANSMITTER_ESTIMATE_ERROR_NO_DELAY.mat");

% plot comparison
figure(7); grid on; hold on;
title("Transmitter position estimate error comparison");
xlabel("time [s]");
ylabel("[m]");
last_step = min([length(te_nod.TRANSMITTER_ESTIMATE_ERROR),length(te.TRANSMITTER_ESTIMATE_ERROR)]);
xlim([0 TIME_STEP*last_step]);
ylim([0 5]);
plot(t_simulation(1:last_step),te_nod.TRANSMITTER_ESTIMATE_ERROR(1:last_step),"Color","blue","LineWidth",1.5);
plot(t_simulation(1:last_step),te.TRANSMITTER_ESTIMATE_ERROR(1:last_step),"Color","red","LineWidth",1.5);
legend("No transmission delay","With transmission delay");
hold off;