client = RemoteAPIClient();
sim = client.getObject('sim');

client.setStepping(true);

sim.startSimulation();
while true
    t = sim.getSimulationTime();
    if t >= 3; break; end
    fprintf('Simulation time: %.2f [s]\n', t);
    client.step();
end
sim.stopSimulation();