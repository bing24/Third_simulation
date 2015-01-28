classdef Simulation

	properties
		time_step
		total_steps
		list_of_operatoring_robots
		list_of_charging_robots
	end
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	methods
		function obj=Simulation()
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		function setTimeStep(obj,time_step)
			obj.time_step=time_step
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		function addOperatingRobots(obj, number_of_robots_to_add)
			for i=1:number_of_robots_to_add
				obj.list_of_operatoring_robots=[obj.list_of_operatoring_robots, OperatingRobot()]
				obj.list_of_operatoring_robots(end).setID(length(obj.list_of_operatoring_robots))
			end
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		function addChargingRobots(obj, number_of_robots_to_add)
			for i=1:number_of_robots_to_add
				obj.list_of_charging_robots=[obj.list_of_charging_robots, OperatingRobot()]
				obj.list_of_charging_robots(end).setID(length(obj.list_of_charging_robots))
			end
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		function simStep(obj)
			for i=1:length(obj.list_of_operatoring_robots)
				obj.list_of_operatoring_robots(i).move(obj.time_step)
			end
			for i=1:length(obj.list_of_charging_robots)
				obj.list_of_charging_robots(i).move(obj.time_step)
			end
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		function simulate(obj)
			for i=1:obj.total_steps
				obj.simStep(obj.time_step)
			end
		end
	end

end