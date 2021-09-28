/**
* Name: Traffic measurement model
* Author: Davide Guastelaa
* Description: 
*/

// voir https://gama-platform.github.io/wiki/Learning-MAS_KMEANS.html
model trafficModel

global {
//number of erroneous assignment of region to vehicles

	// todo effettivamente imparo? Devo misurare l'errore per unità di tempo 
	// e verificare se questo diminuisce.
	int erroneousRegionAssignments <- 0;
	int NUM_TRAIN_AGENTS <- 2;

	//the size of data windows
	int ACW_SIZE <- 10;
	bool WRITE_TO_CSV <- false;
	float min_speed <- 1.0 #km / #h;
	float max_speed <- 5.0 #km / #h;
	int NUM_ANTENNA <- 10;
	int NUM_VEHICLES <- 10;
	int MIN_DISTANCE_ALLOWED <- 500;
	int num_neighbours <- 4 among: [4, 8];
	topology w;
	string csv_output_data_file <- "output.csv";

	/**
	 * The frequency at which vehicles send I am alive messages to antennas
	 */
	int VEHICLES_SEND_MSG_FREQ <- 5 #cycle;
	list<int> VEHICLES_SEND_MSG_FREQs <- range(1, 10);
	/**
	 * The frequency at which antennas measure traffic density based on the received messages
	 */
	int ANTENNA_MEASURE_FREQ <- 10 #cycle;

	//Shapefile of the buildings
	file building_shapefile <- file("../includes/buildings2.shp");
	//Shapefile of the roads
	file road_shapefile <- file("../includes/roads.shp");
	//Shape of the environment
	geometry shape <- envelope(road_shapefile);
	//Step value
	float step <- 10 #s;
	//Graph of the road network
	graph road_network;
	//Map containing all the weights for the road network graph
	map<road, float> road_weights;

	init {
		w <- topology(self);
		//Initialization of the building using the shapefile of buildings
		create building from: building_shapefile;
		//Initialization of the road using the shapefile of roads
		create road from: road_shapefile;

		// print the size of the envelope containing the roads
		write string("geometry size. WIDTH: ", shape.width, " HEIGHT:", shape.height);

		//TODO POSSO CREARE DEGLI AGENTI DI TRAIN, con un boolean
		// i loro valori saranno memorizzati dalle antenne senza alcun 


		//Creation of the people agents
		create people number: NUM_VEHICLES {
			speed <- rnd(min_speed, max_speed);
			//People agents are located anywhere in one of the building
			location <- any_location_in(one_of(building));
			myUpdateRate <- (1 among VEHICLES_SEND_MSG_FREQs at 0) #sec;
			write string("frequency: ", myUpdateRate);
		}

		create people number: NUM_TRAIN_AGENTS {
			isTrainAgent <- true;
			speed <- rnd(min_speed, max_speed);
			//People agents are located anywhere in one of the building
			location <- any_location_in(one_of(building));
			myUpdateRate <- (1 among VEHICLES_SEND_MSG_FREQs at 0) #sec;
			write string("frequency: ", myUpdateRate);
		}

		//Creation of the people agents
		create antenna number: NUM_ANTENNA {
		//People agents are located anywhere in one of the building
			location <- any_location_in(one_of(road));
			write string(self, " at ", location);
		}

		//Weights of the road
		road_weights <- road as_map (each::each.shape.perimeter);
		road_network <- as_edge_graph(road);
	}
	//Reflex to update the speed of the roads according to the weights
	reflex update_road_speed {
		road_weights <- road as_map (each::each.shape.perimeter / each.speed_coeff);
		road_network <- road_network with_weights road_weights;
	}

}

species antenna {

/**  This map contains the messages received by people agents. Each record is 
	 * identified by an istance of people species, and a list of AmAliveMessages. 
	 * This map in particular contains the time series that for "sure" are related
	 * to people that are situated in the local part of environment observed by this
	 * antenna 
	 */

// IS THIS A MAP FOR SURE? I THINK THAT IF THIS CONTAINS THE SERIES 
// SENT FROM AGENTS THAT HAVE BEEN IN THIS REGION, IT IS NOT USEFUL TO
// CONSIDER THE SENDER. THEREFORE, A LIST WOULD BE BETTER

//map<people, list<AmAliveMessage>> theMap;
	list<list<AmAliveMessage>> beliefs;

	/**
	 * While people species send messages to this antenna,
	 * messages are stored temporally in this map while antennas
	 * decide if an agent is or not in their region. Once the decision
	 * is made, the buffer is cleared and the series is added to theMap 
	 * only if the people agent is situated in its region. 
	 */
	map<people, list<AmAliveMessage>> bufferMap;

	/**
	 * The probability that a given people is inside my region
	 */
	map<people, float> agInsideRegionProbability;

	// The color to be used to color antenna's Voronoi region
	rgb regionColor <- rgb(rnd(100, 200), rnd(100, 200), rnd(100, 200), 0.5);
	list<AmAliveMessage> messages_queue;
	int receivedIAmAliveMessages <- 0;
	//rgb color <- #green update: rgb(255 * (receivedIAmAliveMessages / 30), 255 * (1 - (receivedIAmAliveMessages / 30)), 0.0);
	rgb color <- #green;

	reflex checkForPeopleInMyRegion {
	/**
		 * The antenna checks if in the buffer map there are information that 
		 * can be compared to those already in agent's belief base (theMap). If so,
		 * the antenna compares the AmAliveMessages to those already in theMap, then 
		 * sends the results of this comparison to the people instance (the key of 
		 * the dictionary).
		 */
		map<people, list<AmAliveMessage>> bf <- bufferMap;
		//remove a set of keys, where the condition is that the corresponding values are ACW of size ACW_SIZE
		bf >>- bf where (length(each) = ACW_SIZE);

		// loop over the agents that sent exactly ACW_SIZE information to this antenna
		loop ag over: bf.keys {
			if (ag.isTrainAgent = true) {
			// ----------------------------------------
			// MESSAGE FROM A TRAINING VEHICLE
			// MESSAGES ARE DIRECTLY ADDED TO ANTENNA'S
			// KNOWLEDGE BASE
			// ----------------------------------------
				beliefs <+ bf[ag];

				//remove the record which key if ag
				bf[] >- ag;
				// ----------------------------------------
			} else {
			// --------------------------------------------
			// MESSAGE FROM A VEHICLE.
			// THE MESSAGE IS TESTED TOWARDS THE KNOWLEDGE
			// OF THE ANTENNA TO DETERMINE IF THE VEHICLE
			// IS IN ITS REGION
			// --------------------------------------------
			//get the messages received by ag
				list<AmAliveMessage> agMsgList <- bf[ag];
				float agMean <- #max_float;
				//compare to this antenna's information
				loop series over: beliefs {
					float seriesMean <- 0.0;
					loop index from: 0 to: length(agMsgList) - 1 {
					// series is a list of AmAliveMessages of size ACW_SIZE
					// loop over the elements of [series]
						list<float> diffs <- [];
						diffs <+ abs(series[index].speed - agMsgList[index].speed);
						diffs <+ abs(series[index].propagationDelay - agMsgList[index].propagationDelay);
						diffs <+ abs(series[index].directionX - agMsgList[index].directionX);
						diffs <+ abs(series[index].directionY - agMsgList[index].directionY);
						seriesMean <- sum(diffs) + seriesMean;
					}

					seriesMean <- seriesMean / length(agMsgList);
					if (seriesMean <= agMean) {
						agMean <- seriesMean;
					}

				}

				// a questo punto ho lo score dell'agente rispetto alle TS di QUESTA antenna
				// dunque chiedo agli altri agenti se per questo agente hanno uno score maggiore

				// se si, allora questo agente NON è nella zona,
				// se no, allora l'agente è nella zona. Aggiungo la serie alla mia BB
				//list<antenna> neighbors <- agents of_species antenna;
				list<antenna> neighbors <- agents of_species antenna where (each.agInsideRegionProbability contains ag);
				neighbors >- self; //remove myself
				if (length(neighbors) > 0) {
					ask neighbors {
						if (self.agInsideRegionProbability[ag] <= agMean) {
						//the agent is more likely to be in neighbors region
						//write("have region self");
							ag.theRegion <- self;
						} else {
						//write("have region myself");
							ag.theRegion <- myself;
						}

					}

				} else {
					ag.theRegion <- self;
				}
				// --------------------------------------------
			}

		}

		// remove all the record which keys are also in bf.
		// I do this because once the sequences compared to 
		// those in antenna's belief base, the people agent is
		// either inside the zone (thus, the sequence is added to
		// bufferMap, or outside the zone (the sequence is ignored)
		bufferMap[] >>- bf.keys;
	}

	reflex aggregation_function when: every(ANTENNA_MEASURE_FREQ) {
	//write 'processed msgs: ' + length(messages_queue);

	/**
		 * 
		 * 	AGGREGATION FUNCTION SUPPOSED TO BE HERE
		 * 
		 */
//Update the color of this antenna. The color goes from green (less traffic) to red (more traffic)
		color <- rgb(255 * (length(messages_queue) / 30), 255 * (1 - (length(messages_queue) / 30)), 0);

		// clear the message queue, as the aggregation of its values has terminated
		messages_queue <- [];
	}

	aspect default {
		draw circle(30) color: color;
	}

}

/**
 * Custom data type used by agents
 */
species AmAliveMessage {
/**
	 * Sender of this message (people)
	 */
	agent sender;
	/**
	 * Receiver of this message (antenna)
	 */
	agent receiver;

	/**
	 * Distance from the nearest antenna
	 */
	float distance;

	/**
	 * The frequency at which the owner vehicle send messages
	 */
	float vehicleMessagesFrequency;

	/**
	 * The propagation delay is the time it takes for one bit to travel from one end of the link to the other. 
	 * The bits travel in the form of electromagnetic signals. The speed at which electromagnetic signals 
	 * propagate is determined by the medium through which they pass. Following is the formula for propagation delay:
     * <br/>
	 * D/S
 	 * <br/>
	 * where D is the distance between sender and receiver over a link, and S is the transmission speed.
	 */
	float propagationDelay;

	/** Transmission delay is the time needed to push all the packet bits on the transmission 
	 * link. It mainly depends upon the size of the data and channel bandwidth (in bps). 
	 * Following is the formula for transmission delay:
	 * <br/>
	 * L/R
 	 * <br/>
	 * where L is the length of the packet and R is the transmission rate. 
	 */
	float transmissionDelay;

	/**
	 * X Direction of this vehicle. Note that this value can be only calculated
	 * when this message is part of a list of messages received by a specific vehicle
	 */
	float directionX;

	/**
	 * Y Direction of this vehicle. Note that this value can be only calculated
	 * when this message is part of a list of messages received by a specific vehicle
	 */
	float directionY;

	/**
	 * x coordinate of the sender at the moment this message is sent
	 */
	float x;

	/**
	 * y coordinate of the sender at the moment this message is sent
	 */
	float y;

	/**
	 * speed of the sender at the moment this message has been sent
	 */
	float speed;
}

//Species to represent the people using the skill moving
species people skills: [moving] {
	antenna previousRegion <- nil;
	antenna theRegion <- nil;
	bool isTrainAgent <- false;
	float myUpdateRate;

	//Target point of the agent
	point target;
	//Probability of leaving the building
	float leaving_proba <- 0.05;
	//Speed of the agent
	float speed; // <- rnd(10,50) #km/#h;    //5 #km / #h;
	rgb color <- rnd_color(255);

	init {
		if (isTrainAgent) {
		}

	}

	//Reflex to leave the building to another building
	reflex leave when: (target = nil) and (flip(leaving_proba)) {
		target <- any_location_in(one_of(building));
	}

	reflex everyMove {
	//write string(self, " train: ", isTrainAgent);
	//write string(self, " location: ", point(location));
		antenna closestAntenna <- antenna closest_to (self);
		float dist <- self distance_to closestAntenna;
		if (WRITE_TO_CSV) {
			save [name, location.x, location.y, speed, dist, closestAntenna.location.x, closestAntenna.location.y] to: csv_output_data_file type: "csv" rewrite: false;
		}

	}

	reflex sendIAmAliveMessage when: every(myUpdateRate) { //every(VEHICLES_SEND_MSG_FREQ) {
	//QUESTION: is this ok? Or the vehicle should send its information only to one aggregator agent?
		antenna closestAntenna <- antenna closest_to (self);
		if (closestAntenna != nil) {
			if (closestAntenna distance_to (self) <= MIN_DISTANCE_ALLOWED) {
				ask closestAntenna {
				//Create a new message containing information on who sends the message,
				// who receives the message (antenna) and the distance between the vehicle
				// and the antenna
					create AmAliveMessage returns: created_msg;
					AmAliveMessage msg <- first(created_msg);
					msg.sender <- myself;
					msg.receiver <- self;
					msg.distance <- self distance_to myself;
					msg.vehicleMessagesFrequency <- myself.myUpdateRate;
					msg.propagationDelay <- msg.distance / (2.4 * 10 ^ 8); //todo which value?
					msg.x <- myself.location.x;
					msg.y <- myself.location.y;
					AmAliveMessage lastMsgOfThisAgent <- messages_queue last_with (each.sender = myself);
					if (lastMsgOfThisAgent = nil) {
						msg.directionX <- 0.0;
						msg.directionY <- 0.0;
					} else {
						msg.directionX <- msg.x - lastMsgOfThisAgent.x;
						msg.directionY <- msg.y - lastMsgOfThisAgent.y;
					}

					//write string("created message: ", msg.sender, "{", (msg.sender as people).myUpdateRate, "} to ", msg.receiver, " dist: " + msg.distance);
					add msg to: messages_queue;
					// Each vehicle sends an I Am Alive message to the nearest antenna
					receivedIAmAliveMessages <- receivedIAmAliveMessages + 1;
					//trace {
					if (bufferMap[myself] = nil) {
						bufferMap[myself] <- [];
					}
					// add the message to the antenna's queue
					bufferMap[myself] << msg;
				}

			}

		}

	}

	reflex onRegionChange when: previousRegion != theRegion {
		
		previousRegion <- theRegion;
		
		antenna closestAntenna <- antenna closest_to (self);
		if (theRegion != nil and closestAntenna != nil) {
			if ((closestAntenna distance_to (self)) != theRegion) {
				erroneousRegionAssignments <- erroneousRegionAssignments + 1;
			}

		}

	}

	//Reflex to move to the target building moving on the road network
	reflex move when: target != nil {
	//we use the return_path facet to return the path followed
		path path_followed <- goto(target: target, on: road_network, recompute_path: false, return_path: true, move_weights: road_weights);
		if (location = target) {
			target <- nil;
		} }

	aspect default {
		if (not isTrainAgent) {
			draw circle(15) color: color;
		} else {
			draw square(15) color: #black;
		}

		if (theRegion != nil) {
			draw line([{location.x, location.y}, {theRegion.location.x, theRegion.location.y}]) width: 8 color: #black;
		}

	} }

	//Species to represent the buildings
species building {

	aspect default {
		draw shape color: #gray;
	}

}
//Species to represent the roads
species road {
//Capacity of the road considering its perimeter
	float capacity <- 1 + shape.perimeter / 30;
	//Number of people on the road
	int nb_people <- 0 update: length(people at_distance 1);
	//Speed coefficient computed using the number of people on the road and the capicity of the road
	float speed_coeff <- 1.0 update: exp(-nb_people / capacity) min: 0.1;
	int buffer <- 3;

	aspect default {
		draw (shape + buffer * speed_coeff) color: #red;
	}

}

grid cell neighbors: num_neighbours use_neighbors_cache: true use_individual_shapes: false use_regular_agents: false parallel: true {
// Note: since GAMA 1.7, the topology needs to be specified for this computation to use continuous distances
	rgb color <- #white update: ((antenna closest_to location) using w).regionColor;

	init {
		color <- blend(color, one_of(neighbors).color, 0.2);
	}

	// use the reflex to blend colors
	//reflex {
	//	color <- blend(color, one_of(neighbors).color, 0.2);
	//}
}

experiment traffic type: gui {
	float minimum_cycle_duration <- 0.2 #sec;
	output {
		display carte type: opengl {
			species building refresh: false;
			species road;
			species people;
			species antenna;
			grid cell;
			
			chart "datalist_bar" type: histogram series_label_position: onchart size: {1.0,0.5} position: {0,1}{
				datalist legend: ["A", "B", "C"] style: bar value: [erroneousRegionAssignments] color: [#green];
			}
		}

		//display "datalist_bar_cchart" type: java2D {
			

		//}

	}

}
