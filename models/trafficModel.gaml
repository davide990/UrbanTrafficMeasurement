/**
* Name: Traffic measurement model
* Author: Davide Guastelaa
* Description: 
*/

// voir https://gama-platform.github.io/wiki/Learning-MAS_KMEANS.html
model trafficModel

global {

//the size of data windows
	int ACW_SIZE <- 2;
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

		//Creation of the people agents
		create people number: NUM_VEHICLES {
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
		bf >>- bf where (length(each) = ACW_SIZE);

		// loop over the agents that sent exactly ACW_SIZE information to this antenna
		loop ag over: bf.keys {
		//get the messages received by ag
			list<AmAliveMessage> agMsgList <- bf[ag];
			float agMean <- 0.0;
			//compare to this antenna's information
			loop series over: beliefs {
				float seriesMean <- 0.0;
				loop index from: 0 to: length(agMsgList) {
				// series is a list of AmAliveMessages of size ACW_SIZE
				// loop over the elements of [series]
					int xdiff <- abs(series[index].x - agMsgList[index].x);
					int ydiff <- abs(series[index].y - agMsgList[index].y);
					float distanceDiff <- abs(series[index].distance - agMsgList[index].distance);
					float speedDiff <- abs(series[index].speed - agMsgList[index].speed);
					seriesMean <- seriesMean + xdiff;
					seriesMean <- seriesMean + ydiff;
					seriesMean <- seriesMean + distanceDiff;
					seriesMean <- seriesMean + speedDiff;
				}

				trace {
					seriesMean <- seriesMean / length(agMsgList);
				}

			}

			//get the most similar sequence

			//send the score to ag

		}

		// remove all the record which keys are also in bf.
		// I do this because once the sequences compared to 
		// those in antenna's belief base, the people agent is
		// either inside the zone (thus, the sequence is added to
		// bufferMap, or outside the zone (the sequence is ignored)
		bufferMap[] >>- bf.keys;
	}

	reflex aggregation_function when: every(ANTENNA_MEASURE_FREQ) {
		write 'processed msgs: ' + length(messages_queue);

		/**
		 * 
		 * 	AGGREGATION FUNCTION SUPPOSED TO BE HERE
		 * 
		 */
//Update the color of this antenna. The color goes from green (less traffic) to red (more traffic)
		color <- rgb(255 * (length(messages_queue) / 30), 255 * (1 - (length(messages_queue) / 30)), 0.0);

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
	 * x coordinate of the sender at the moment this message is sent
	 */
	int x;

	/**
	 * y coordinate of the sender at the moment this message is sent
	 */
	int y;

	/**
	 * speed of the sender at the moment this message is sent
	 */
	float speed;
}

//Species to represent the people using the skill moving
species people skills: [moving] {
	float myUpdateRate;

	//Target point of the agent
	point target;
	//Probability of leaving the building
	float leaving_proba <- 0.05;
	//Speed of the agent
	float speed; // <- rnd(10,50) #km/#h;    //5 #km / #h;
	rgb color <- rnd_color(255);
	//Reflex to leave the building to another building
	reflex leave when: (target = nil) and (flip(leaving_proba)) {
		target <- any_location_in(one_of(building));
	}

	reflex everyMove {
		write string(self, " location: ", point(location));
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
					write string("created message: ", msg.sender, "{", (msg.sender as people).myUpdateRate, "} to ", msg.receiver, " dist: " + msg.distance);
					add msg to: messages_queue;
					// Each vehicle sends an I Am Alive message to the nearest antenna
					receivedIAmAliveMessages <- receivedIAmAliveMessages + 1;
					//trace {
					if (bufferMap[myself] = nil) {
						bufferMap[myself] <- [];
					}
					// add the message to the antenna's queue
					bufferMap[myself] << msg;
					//}

				}

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
		draw circle(15) color: color;
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
		}

	}

}
