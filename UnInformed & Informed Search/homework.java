package AI;

import java.io.*;
import java.util.*;


class EdgeInfo{
	int source;
	int destination;
	int cost;
	public EdgeInfo(){
		cost = 1;
	}
}

class VertexInfo{
	int number;
	int heuristic;
	public VertexInfo(){
		number = 0;
		heuristic = 0;
	}
}

public class homework {
	
	private static final String INPUT_FILE_NAME = "/Users/Ankur/Desktop/input.txt";
	private static final String OUTPUT_FILE_NAME = "/Users/Ankur/Desktop/output.txt";

	static int nodeCount = -1;//counter for vertex numbering
	private static HashMap<String, VertexInfo> nodeMap = new HashMap<String, VertexInfo>();
	private static List<String> result = new ArrayList<String>();

	private static void createNodeMap(String nodeName){
		if(!nodeMap.containsKey(nodeName)){
			VertexInfo v = new VertexInfo();
			v.number = ++nodeCount;
			nodeMap.put(nodeName, v);
		}
	}
	
	private static void createNodeMap(String nodeName, int heuristic){
		if(nodeMap.containsKey(nodeName)){
			VertexInfo v = nodeMap.get(nodeName);
			v.heuristic = heuristic;
			nodeMap.put(nodeName, v);
		}
	}
	public static void main(String[] args){

		try {
			File file = new File(INPUT_FILE_NAME);
	        BufferedReader in = new BufferedReader(new FileReader(file));
			String algo = in.readLine();
			String source = in.readLine();
			String destination = in.readLine();
			if(source.equals(destination)){
				result.add(source + " 0");
				System.out.print(source + " 0");
			}
			else{
			
				//Add normal edges
				int numLive = Integer.parseInt(in.readLine());
				String lineLive;
				ArrayList<EdgeInfo> edges = new ArrayList<EdgeInfo>();
				
				for (int i = 0; i < numLive; i++){
					lineLive = in.readLine();
					String[] split = lineLive.split(" ");
					
					createNodeMap(split[0]);
					createNodeMap(split[1]);
					
					EdgeInfo edgeInfo = new EdgeInfo();
					edgeInfo.source = nodeMap.get(split[0]).number;
					edgeInfo.destination = nodeMap.get(split[1]).number;
					edgeInfo.cost = Integer.parseInt(split[2]);
					edges.add(edgeInfo);
				}	
				
				Graph g = new Graph(true);
				int start = nodeMap.get(source).number;
				int end = nodeMap.get(destination).number;

				switch(algo){
					case "BFS": 
				        g.initializeGraph(nodeMap, edges, false);
				        g.bfs(start, end);
				        result = g.printPath(start, end);
						break;
					case "DFS":
				        g.initializeGraph(nodeMap, edges, true);
				        g.dfs(start, end);
				        result = g.printPath(start, end);
						break;
					case "UCS":
						g.initializeGraph(nodeMap, edges, false);
						g.ucs(start, end);
						result = g.printPath(start, end);
						break;
					case "A*":
						// Add Sunday metadata
						int numSunday = Integer.parseInt(in.readLine());
						String lineSunday;
						
						for (int i = 0; i < numSunday; i++){
							lineSunday = in.readLine();
							String[] split = lineSunday.split(" ");
							
							createNodeMap(split[0], Integer.parseInt(split[1]));
						}	
						g.initializeGraph(nodeMap, edges, false);
						g.AStar(start, end);
						result = g.printPath(start, end);
						break;
					default:
						break;
				}
			}
				
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		// generate output
 		try{
         	BufferedWriter bw = new BufferedWriter(new FileWriter(OUTPUT_FILE_NAME));
         	 for (String line : result){
         		 bw.write(line);
         		 bw.newLine();
         	 }
         
         	bw.close();
        }catch(IOException e){e.printStackTrace();}
	
	}
}

class Graph {

    //------------------------------------------------------------
    // Utility classes and enumerations.
    //------------------------------------------------------------

    //Represents an edge in the adjacency list.
    private class Edge {
        public int label;
        public int cost;
        public Edge next;
    }

    //State enumeration for vertices during traversal.
    public static enum State {
        Undiscovered, Discovered, Processed
    }

    //Represents a vertex in the graph.
    private class Vertex {
    	public int number;
    	public String name;
        public int parent;
        public State state;
        public int pathCost;
        public int heuristic;
        public Vertex(int number, String name, int heuristic) {
            state = State.Undiscovered;
            parent = -1;
            pathCost = 0;
            this.number = number;
            this.heuristic = heuristic;
            this.name = name;
        }
    }


    public Graph(boolean directed) {
        this.directed = directed;
    }

    //------------------------------------------------------------
    // Data members representing graph state.
    //------------------------------------------------------------

    private boolean directed;    
    private Edge[] adjacencyList;
    private Vertex[] vertices;
	private List<String> result = new ArrayList<String>();


	//------------------------------------------------------------
    // Graph initialization and population functions.
    //------------------------------------------------------------

    //Initialize the graph from a given file.
    public void initializeGraph(HashMap<String, VertexInfo> nodeMap, 
    		ArrayList<EdgeInfo> edges,  boolean addToFront) {
    	
    	
        //Initialize the vertices.
        int vertexCount = nodeMap.size();
        vertices = new Vertex[vertexCount];
        for (Map.Entry<String,VertexInfo> entry : nodeMap.entrySet()) {
        	String name = entry.getKey();
        	VertexInfo vertexInfo = entry.getValue();
        	int vertexNumber = vertexInfo.number;
        	vertices[vertexNumber] = new Vertex(vertexNumber, name, vertexInfo.heuristic);
        }
        
        //Initialize the adjacency list.
        adjacencyList = new Edge[vertexCount];

        for(EdgeInfo edge: edges){
        	addEdge(edge.source, edge.destination, edge.cost, addToFront, directed);
        }
    }

   
    //Add an edge to the graph.
    public void addEdge(int source, int dest, int cost, boolean addToFront,
            boolean isDirected) {

        Edge edge = new Edge();
        edge.label = dest;
        edge.cost = cost;
        edge.next = null;
        if(addToFront){
	        edge.next = adjacencyList[source];
	        adjacencyList[source] = edge;
        }
        else{
       
	        Edge e = adjacencyList[source];
	        if(e==null)
	        	adjacencyList[source] = edge;
	        else{
		        while(e.next!=null){
		        	e = e.next;
		        }
		        e.next = edge;
		        adjacencyList[source] = e;
	        }
        }

        //If this is an undirected graph, add the reverse edge.
        if (!isDirected) {
            addEdge(dest, source, cost, addToFront, true);
        }
    }

    //------------------------------------------------------------
    // BFS algorithm implementation.
    //------------------------------------------------------------

    public void bfs(int startNode, int endNode) {
    	

        Queue<Integer> queue = new LinkedList<Integer>();
        vertices[startNode].state = State.Discovered;
        queue.add(startNode);

        while (!queue.isEmpty()) {

            //Take the queued vertex 
            int index = queue.poll();
              
            //get first edge.
            Edge edge = adjacencyList[index];

            //Loop over all of the current vertex's edges.
            while(edge != null) {
            	
                int dest = edge.label;
                Vertex child = vertices[dest];
                if (child.state == State.Undiscovered) {
                    queue.add(dest);
                    child.parent = index;
                    child.pathCost = vertices[index].pathCost + 1;// level = parent's level + 1
                    child.state = State.Discovered;
                }
                
                edge = edge.next;
            }

            if(index == endNode)
            	break;
        }
    }
    
    //------------------------------------------------------------
    // DFS algorithm implementation.
    //------------------------------------------------------------

    public void dfs(int startNode, int endNode) {

        //Create a stack and initialize it with the start node.
        Stack<Integer> stack = new Stack<Integer>();
        vertices[startNode].state = State.Discovered;
        stack.add(startNode);

        while (!stack.isEmpty()) {

            int index = stack.pop();
            
            Edge edge = adjacencyList[index];

            //Loop over all of the current vertex's edges.
            while(edge != null) {
            	
                int dest = edge.label;
                Vertex child = vertices[dest];
                if (child.state == State.Undiscovered) {
                    stack.add(dest);
                    child.parent = index;
                    
                    child.pathCost = vertices[index].pathCost + 1; // level = parent's level + 1
                    child.state = State.Discovered;
                }

                //Move on to the next edge.
                edge = edge.next;
            }
            
            if(index == endNode)
            	break;
        }
    }
    
    //------------------------------------------------------------
    // UCS algorithm implementation.
    //------------------------------------------------------------

    public void ucs(int startNode, int endNode) {
    	
        //Create a priority queue and initialize it with the start node.
        PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>(20, 
            new Comparator<Vertex>(){

                //override compare method
                public int compare(Vertex i, Vertex j){
                    if(i.pathCost > j.pathCost){
                        return 1;
                    }

                    else if (i.pathCost < j.pathCost){
                        return -1;
                    }

                    else{
                        return 0;
                    }
                }
        	}
        );
        Vertex source = vertices[startNode];
        source.state = State.Discovered;
        queue.add(source);

        while (!queue.isEmpty()) {

            //Take the queued vertex 
            Vertex current = queue.poll();
//            System.out.println("current: " + current.label + " pathCost: " + current.pathCost + " parent: " + current.parent);
            
            Edge edge = adjacencyList[current.number];

            //Loop over all of the current vertex's edges.
            while(edge != null) {
            	

                int dest = edge.label;
                Vertex child = vertices[dest];
                
                int new_cost = current.pathCost + edge.cost;

                if (child.state == State.Undiscovered && !queue.contains(child)) {
                    child.pathCost = new_cost;
                	child.parent = current.number;
                    queue.add(child);
                }
                else if((queue.contains(child) || child.state == State.Discovered) 
                		&& (child.pathCost > new_cost)){

                	child.parent = current.number;
                	child.pathCost = new_cost;
                    // the next two calls decrease the key of the child in the queue
                    queue.remove(child);
                    queue.add(child);
                }
                edge = edge.next;
            }
            	
            //explored all of its adjacent vertices.
            current.state = State.Discovered;

        }
    }
    
    //------------------------------------------------------------
    // A* algorithm implementation.
    //------------------------------------------------------------

    public void AStar(int startNode, int endNode) {
    	
        //Create a priority queue and initialize it with the start node.
        PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>(20, 
            new Comparator<Vertex>(){

                //override compare method
                public int compare(Vertex i, Vertex j){
                    if(i.pathCost > j.pathCost){
                        return 1;
                    }

                    else if (i.pathCost < j.pathCost){
                        return -1;
                    }

                    else{
                        return 0;
                    }
                }
        	}
        );
        Vertex source = vertices[startNode];
        source.state = State.Discovered;
        queue.add(source);

        while (!queue.isEmpty()) {

            //Take the queued vertex 
            Vertex current = queue.poll();
            current.state = State.Discovered;
            
//            System.out.println("current: " + current.label + " pathCost: " + current.pathCost + " parent: " + current.parent);
            if(current.number == endNode)
        		break; //goal found
            
            Edge edge = adjacencyList[current.number];

            //Loop over all of the current vertex's edges.
            while(edge != null) {
            	

                int dest = edge.label;
                
                Vertex child = vertices[dest];
                
                int new_edge_cost = current.pathCost + edge.cost;
                int new_cost = new_edge_cost + child.heuristic;
                int child_cost = child.pathCost + child.heuristic;
                if (child.state == State.Undiscovered && !queue.contains(child)) {
                    child.pathCost = new_edge_cost;
                	child.parent = current.number;
                    queue.add(child);
                }
                else if((queue.contains(child) || child.state == State.Discovered) 
                		&& (child_cost > new_cost)){

                	child.parent = current.number;
                	child.pathCost = new_edge_cost;
                    // the next two calls decrease the key of the child in the queue
                    queue.remove(child);
                    queue.add(child);
                }
                edge = edge.next;
            }
            	
            

        }
    }

    //------------------------------------------------------------
    // Function to print path to a node
    //------------------------------------------------------------
    public void printPathTo(int dest) {
    	
        if (dest == -1) return;
        Vertex vertex = vertices[dest];
        
        printPathTo(vertex.parent);

        System.out.print(vertex.name + " " + vertex.pathCost + "\n");
        result.add(vertex.name + " " + vertex.pathCost);
        
        
    }

    public List<String> printPath(int source, int destination) {
        System.out.print("Path from  " + source +" to node " + destination + ":\n");
        printPathTo(destination);
        System.out.println("");
        return result;
    }

    
}