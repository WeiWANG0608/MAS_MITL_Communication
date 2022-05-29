import java.awt.geom.Point2D;
import com.uppaal.engine.CannotEvaluateException;
import com.uppaal.engine.Engine;
import com.uppaal.engine.EngineException;
import com.uppaal.engine.EngineStub;
import com.uppaal.engine.Parser;
import com.uppaal.engine.Problem;
import com.uppaal.engine.QueryFeedback;
import com.uppaal.engine.QueryResult;
import com.uppaal.model.core2.Data2D;
import com.uppaal.model.core2.DataSet2D;
import com.uppaal.model.core2.Document;
import com.uppaal.model.core2.Edge;
import com.uppaal.model.core2.Location;
import com.uppaal.model.core2.Property;
import com.uppaal.model.core2.PrototypeDocument;
import com.uppaal.model.core2.Query;
import com.uppaal.model.core2.QueryData;
import com.uppaal.model.core2.Template;
import com.uppaal.model.system.SystemEdge;
import com.uppaal.model.system.SystemLocation;
import com.uppaal.model.system.symbolic.SymbolicState;
import com.uppaal.model.system.symbolic.SymbolicTransition;
import com.uppaal.model.system.symbolic.SymbolicTrace;
import com.uppaal.model.system.concrete.Limit;
import com.uppaal.model.system.concrete.ConcreteVariable;
import com.uppaal.model.system.concrete.ConcreteState;
import com.uppaal.model.system.concrete.ConcreteTransitionRecord;
import com.uppaal.model.system.concrete.ConcreteTrace;
import com.uppaal.model.system.UppaalSystem;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.FileReader;
import java.io.IOException;
import java.math.BigDecimal;
import java.net.MalformedURLException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.*;
import java.util.concurrent.TimeUnit;
import java.text.*;
import java.time.*;
import java.time.format.DateTimeFormatter;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class MITL_verification {
    /**
     * Valid kinds of labels on locations.
     */
    public enum LKind {
        name, init, urgent, committed, invariant, exponentialrate, comments
    };
    /**
     * Valid kinds of labels on edges.
     */
    public enum EKind {
        select, guard, synchronisation, assignment, comments
    };
    /**
     * Sets a label on a location.
     * @param l the location on which the label is going to be attached
     * @param kind a kind of the label
     * @param value the label value (either boolean or String)
     * @param x the x coordinate of the label
     * @param y the y coordinate of the label
     */
    public static void setLabel(Location l, LKind kind, Object value, int x, int y) {
        l.setProperty(kind.name(), value);
        Property p = l.getProperty(kind.name());
        p.setProperty("x", x);
        p.setProperty("y", y);
    }
    /**
     * Adds a location to a template.
     * @param t the template
     * @param name a name for the new location
     * @param exprate an expression for an exponential rate
     * @param x the x coordinate of the location
     * @param y the y coordinate of the location
     * @return the new location instance
     */
    public static Location addLocation(Template t, String name, String exprate,
									   int x, int y)
    {
        Location l = t.createLocation();
        t.insert(l, null);
        l.setProperty("x", x);
        l.setProperty("y", y);
		if (name != null)
			setLabel(l, LKind.name, name, x, y-28);
		if (exprate != null)
			setLabel(l, LKind.exponentialrate, exprate, x, y-28-12);
        return l;
    }
    /**
     * Sets a label on an edge.
     * @param e the edge
     * @param kind the kind of the label
     * @param value the content of the label
     * @param x the x coordinate of the label
     * @param y the y coordinate of the label
     */
    public static void setLabel(Edge e, EKind kind, String value, int x, int y) {
        e.setProperty(kind.name(), value);
        Property p = e.getProperty(kind.name());
        p.setProperty("x", x);
        p.setProperty("y", y);
    }
    /**
     * Adds an edge to the template
     * @param t the template where the edge belongs
     * @param source the source location
     * @param target the target location
     * @param guard guard expression
     * @param sync synchronization expression
     * @param update update expression
     * @return
     */
    public static Edge addEdge(Template t, Location source, Location target,
							   String guard, String sync, String update)
    {
        Edge e = t.createEdge();
        t.insert(e, null);
        e.setSource(source);
        e.setTarget(target);
        int x = (source.getX()+target.getX())/2;
        int y = (source.getY()+target.getY())/2;
        if (guard != null) {
            setLabel(e, EKind.guard, guard, x-15, y-28);
        }
        if (sync != null) {
            setLabel(e, EKind.synchronisation, sync, x-15, y-14);
        }
        if (update != null) {
            setLabel(e, EKind.assignment, update, x-15, y);
        }
        return e;
    }

    public static void print(UppaalSystem sys, SymbolicState s) {
        System.out.print("(");
		boolean first = true;
        for (SystemLocation l: s.getLocations()) {
			if (first) first=false; else System.out.print(", ");
            System.out.print(l.getName());
        }
        int val[] = s.getVariableValues();
        for (int i=0; i<sys.getNoOfVariables(); i++) {
			System.out.print(", ");
            System.out.print(sys.getVariableName(i)+"="+val[i]);
        }
        List<String> constraints = new ArrayList<>();
        s.getPolyhedron().getAllConstraints(constraints);
        for (String cs : constraints) {
			System.out.print(", ");
            System.out.print(cs);
        }
        System.out.println(")");
    }

	static final BigDecimal zero = BigDecimal.valueOf(0);

	public static void print(UppaalSystem sys, ConcreteState s) {
        System.out.print("(");
		boolean first = true;
        for (SystemLocation l: s.getLocations()) {
			if (first) first=false; else System.out.print(", ");
            System.out.print(l.getName());
        }
		Limit limit = s.getInvariant();
		if (!limit.isUnbounded()) {
			if (limit.isStrict())
				System.out.print(", limit<");
			else
				System.out.print(", limit≤");
			System.out.print(limit.getDoubleValue());
		}
        ConcreteVariable val[] = s.getCVariables();
		int vars = sys.getNoOfVariables();
        for (int i=0; i<vars; ++i) {
			System.out.print(", ");
            System.out.print(sys.getVariableName(i)+"="+val[i].getValue(zero));
        }
        for (int i=0; i<sys.getNoOfClocks(); i++) {
			System.out.print(", ");
            System.out.print(sys.getClockName(i)+"="+val[i+vars].getValue(zero));
			System.out.print(", ");
            System.out.print(sys.getClockName(i)+"'="+val[i+vars].getRate());
        }
        System.out.println(")");
    }

    public static Document createSampleModel(String docsystem, String docdecl, String[] templist, String[] tempdecl, 
                                            String[][][] locations, String[][] labels, String[][][] edges, int[] initalstate)
    {

		// create a new Uppaal model with default properties:
		Document doc = new Document(new PrototypeDocument());    
		// add global variables:
		doc.setProperty("declaration", docdecl);
        Template[] templatesList = new Template[templist.length];

        for(int i=0; i<templist.length; i++){
            // add a TA template:
            templatesList[i] = doc.createTemplate();
            doc.insert(templatesList[i], null);
            templatesList[i].setProperty("name", templist[i]);
            if ( tempdecl[i] != null && tempdecl[i] != "null"){
                // System.out.println("decl is "+ i + tempdecl[i] +"  "+tempdecl[i].length());
                templatesList[i].setProperty("declaration", tempdecl[i]);
            }
            
            // the template has initial location:
            Location[] locationList = new Location[locations[i].length];
            for(int j=0; j<locations[i].length; j++){
                locationList[j] = addLocation(templatesList[i], locations[i][j][0], locations[i][j][1], Integer.parseInt(locations[i][j][2].replace("\"", "")), Integer.parseInt(locations[i][j][3].replace("\"", "")));
                // System.out.println("AddLocation "+ locations[i][j][0] +" "+locations[i][j][1] +" "+ locations[i][j][2] +" "+locations[i][j][3]);
                if(j==initalstate[i]){
                    locationList[j].setProperty("init", true);
                }
                if(labels[i][j] != null && labels[i][j] != "null"){
                    Object obj_label = labels[i][j];
                    setLabel(locationList[j], LKind.invariant, obj_label, locationList[j].getX()-7, locationList[j].getY()+10);
                    // System.out.println("Location "+locationList[j].getName() +" "+ labels[i][j] +" "+ String.valueOf(locationList[j].getX()-7) +" "+ String.valueOf(locationList[j].getY()+1) );
                }
            }
            for(int k=0; k<edges[i].length; k++){
                // add edges
                for(int k1=0; k1<locations[i].length; k1++){
                    for(int k2=0; k2<locations[i].length; k2++){
                        if(locationList[k1].getName().equals(edges[i][k][0]) && locationList[k2].getName().equals(edges[i][k][1])){
                         addEdge(templatesList[i], locationList[k1], locationList[k2], edges[i][k][2], edges[i][k][3], edges[i][k][4]);
                         // System.out.println("AddEdge "+locationList[k1].getName() +" "+ locationList[k2].getName() +" "+ edges[i][k][2] +" "+ edges[i][k][3] +" "+ edges[i][k][4]);
                        }
                    }
                }
            }
        }

		// add system declaration:
		doc.setProperty("system",docsystem);
		return doc;
    }

    public static Document loadModel(String location) throws IOException
    {
		try {
			// try URL scheme (useful to fetch from Internet):
			return new PrototypeDocument().load(new URL(location));
		} catch (MalformedURLException ex) {
			// not URL, retry as it were a local filepath:
			return new PrototypeDocument().load(new URL("file", null, location));
		}
    }

    public static Engine connectToEngine() throws EngineException, IOException
    {
		String os = System.getProperty("os.name");
		String here = System.getProperty("user.dir");
		String path = null;
		if ("Linux".equals(os)) {
			path = here+"/bin-Linux/server";
		} else if ("Mac OS X".equals(os)) {
			path = here+"/bin-Darwin/server";
		} else if ("Windows".equals(os)) {
			path = here+"\\bin-Windows\\server.exe";
		} else {
			System.err.println("Unknown operating system.");
			System.exit(1);
		}
		Engine engine = new Engine();
		engine.setServerPath(path);
		engine.setServerHost("localhost");
		engine.setConnectionMode(EngineStub.BOTH);
		engine.connect();
		return engine;
    }

    public static UppaalSystem compile(Engine engine, Document doc)
		throws EngineException, IOException
    {
		// compile the model into system:
		ArrayList<Problem> problems = new ArrayList<>();
		UppaalSystem sys = engine.getSystem(doc, problems);
		if (!problems.isEmpty()) {
			boolean fatal = false;
			System.out.println("There are problems with the document:");
			for (Problem p : problems) {
				System.out.println(p.toString());
				if (!"warning".equals(p.getType())) { // ignore warnings
					fatal = true;
				}
			}
			if (fatal) {
				System.exit(1);
			}
		}
		return sys;
    }

	public static void printTrace(UppaalSystem sys, SymbolicTrace trace)
	{
		if (trace == null) {
			System.out.println("(null trace)");
			return;
		}
		Iterator<SymbolicTransition> it = trace.iterator();
		print(sys, it.next().getTarget());
		while (it.hasNext()) {
			SymbolicTransition tr = it.next();
			if (tr.getSize()==0) {
				// no edges, something special (like "deadlock" or initial state):
				System.out.println(tr.getEdgeDescription());
			} else {
				// one or more edges involved, print them:
				boolean first = true;
				for (SystemEdge e: tr.getEdges()) {
					if (first) first = false; //else System.out.print(", ");
					// System.out.print(e.getProcessName()+": "
					// 				 + e.getEdge().getSource().getPropertyValue("name")
					// 				 + " \u2192 "
					// 				 + e.getEdge().getTarget().getPropertyValue("name"));
				}
			}
			System.out.println();
			print(sys, tr.getTarget());
		}
		System.out.println();
	}

	public static void printTrace(UppaalSystem sys, ConcreteTrace trace)
	{
		if (trace == null) {
			System.out.println("(null trace)");
			return;
		}
		Iterator<ConcreteTransitionRecord> it = trace.iterator();
		print(sys, it.next().getTarget());
		while (it.hasNext()) {
			ConcreteTransitionRecord tr = it.next();
			if (tr.getSize()==0) {
				// no edges, something special (like "deadlock" or initial state):
				System.out.println(tr.getTransitionDescription());
			} else {
				// one or more edges involved, print them:
				boolean first = true;
				for (SystemEdge e: tr.getEdges()) {
					if (first) first = false; else System.out.print(", ");
					System.out.print(e.getProcessName()+": "
									 + e.getEdge().getSource().getPropertyValue("name")
									 + " \u2192 "
									 + e.getEdge().getTarget().getPropertyValue("name"));
				}
			}
			System.out.println();
			print(sys, tr.getTarget());
		}
		System.out.println();
	}

	static public void print(QueryData data) {
		for (String title: data.getDataTitles()) {
			DataSet2D plot = data.getData(title);
			System.out.println("Plot \""+plot.getTitle()+
							   "\" showing \"" + plot.getYLabel() +
							   "\" over \"" + plot.getXLabel()+"\"");
			for (Data2D traj: plot) {
				System.out.print("Trajectory " + traj.getTitle()+":");
				for (Point2D.Double p: traj)
					System.out.print(" ("+p.x+","+p.y+")");
				System.out.println();
			}
		}
	}

    public static SymbolicTrace symbolicSimulation(Engine engine,
													UppaalSystem sys)
		throws EngineException, IOException, CannotEvaluateException
    {
		SymbolicTrace trace = new SymbolicTrace();
		// compute the initial state:
		SymbolicState state = engine.getInitialState(sys);
		// add the initial transition to the trace:
		trace.add(new SymbolicTransition(null, null, state));
		int simu = 10;
		int si = 0;
		while (state != null && si<simu) {
			// print(sys, state);
			// compute the successors (including "deadlock"):
			ArrayList<SymbolicTransition> trans = engine.getTransitions(sys, state);
			// select a random transition:
			int n = (int)Math.floor(Math.random()*trans.size());
			SymbolicTransition tr = trans.get(n);
			// check the number of edges involved:
			if (tr.getSize()==0) {
				// no edges, something special (like "deadlock"):
				System.out.print(tr.getEdgeDescription());
			} else {
				// one or more edges involved, print them:
				boolean first = true;
				for (SystemEdge e: tr.getEdges()) {
					if (first) first = false; else System.out.print(", ");
					// System.out.print(e.getProcessName()+": "
					// 				 + e.getEdge().getSource().getPropertyValue("name")
					// 				 + " \u2192 "
					// 				 + e.getEdge().getTarget().getPropertyValue("name"));
				}
			}
			//System.out.println();
			// jump to a successor state (null in case of deadlock):
			state = tr.getTarget();
			// if successfull, add the transition to the trace:
			if (state != null)
				trace.add(tr);
			si+=1;
		}
        return trace;
    }


    public static void saveXTRFile(SymbolicTrace trace, String file)
		throws IOException
    {
		/* BNF for the XTR format just in case
		   (it may change, thus don't rely on it)
		   <XTRFomat>  := <state> ( <state> <transition> ".\n" )* ".\n"
		   <state>     := <locations> ".\n" <polyhedron> ".\n" <variables> ".\n"
		   <locations> := ( <locationId> "\n" )*
		   <polyhedron> := ( <constraint> ".\n" )*
		   <constraint> := <clockId> "\n" clockId "\n" bound "\n"
		   <variables> := ( <varValue> "\n" )*
		   <transition> := ( <processId> <edgeId> )* ".\n"
		*/
		FileWriter out = new FileWriter(file);
		Iterator<SymbolicTransition> it = trace.iterator();
		it.next().getTarget().writeXTRFormat(out);
		while (it.hasNext()) {
			it.next().writeXTRFormat(out);
		}
		out.write(".\n");
		out.close();
    }

	static SymbolicTrace strace = null;
	static ConcreteTrace ctrace = null;

    /**
     * @param args the command line arguments
     * @throws Exception
     */



    public static void main(String[] args) throws Exception {
        if (args.length<1) {
            System.out.println("This is a demo of Uppaal model.jar API");
            System.out.println("Use one of the following arguments:");
            System.out.println("  hardcoded");
            System.out.println("  <URL>");
            System.out.println("  <path/file.xml>");
            System.exit(0);
        }
        String declaration2verify = "";

        DateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd");
        Date date = new Date();
        String strDate = dateFormat.format(date); 
        String direction = "/home/wei/PycharmProjects/MITL_communication/MITL_communication/files";

        Path pathToFlag = Paths.get(direction + "/"+strDate+"/flag.txt");
        String flag = Files.readString(pathToFlag);
        System.out.println("===== Flag =====");
        System.out.println(flag);
        System.out.println(direction + "/"+strDate+"/system_info" + flag + ".txt");

        try {
			com.uppaal.model.io2.XMLReader.setXMLResolver(new com.uppaal.model.io2.UXMLResolver());
			Document doc;
			if ("hardcoded".equals(args[0])) {
				// create a hardcoded model:

 
                Path pathToFile = Paths.get(direction + "/"+strDate+"/system_info" + flag + ".txt");
				String content = Files.readString(pathToFile);
				// System.out.print(content);
				String[] parts = content.split("&");

                String docsystem= parts[0];
                System.out.println("===== System =====");
                System.out.println(docsystem);

                String docdecl = parts[1];
                System.out.println("===== Declarations =====");
                System.out.print(docdecl);
                declaration2verify = docdecl;

                String[] dec = parts[2].split("#");
                System.out.println("===== Templates declarations =====");
                String[] tempdecl = new String[dec.length];
                // System.out.println(tempdecl[0]+" " +tempdecl[1]+"\n");
                for (int i=1; i<dec.length; i++){
                    String dc = dec[i].trim().replace("[", "").replace("]", "").replace("\"", "");                    
                    tempdecl[i] = "null".equals(dc) ? null :  dc;
                }
                System.out.println(tempdecl[0]+"\n" +tempdecl[1]+"\n");
                

                String[] templist = parts[3].split("#");
                System.out.println("===== Templates =====");
                for (int i=1; i<templist.length; i++){
                    templist[i] = "null".equals(templist[i].replace("\"", "")) ? null : templist[i];
                }
                System.out.println(templist[0]+" " +templist[1]+"\n");

                int[] inital_state = {0, 0}; 


                JSONParser jsonParser = new JSONParser();
               
                //Parsing the contents of the JSON file
                JSONObject jsonObject = (JSONObject) jsonParser.parse(new FileReader(direction + "/"+strDate+"/system_data" + flag + ".json"));
                //JSONArray jsonAr = (JSONArray) jsonParser.parse(new FileReader("/home/wei/uppaal64-4.1.26/demo/ModelDemo/src/system_data.json"));

                JSONObject jsonObject_lo = (JSONObject) jsonObject.get("locations");
                JSONObject jsonObject_la = (JSONObject) jsonObject.get("labels");
                JSONObject jsonObject_ed = (JSONObject) jsonObject.get("edges");

                JSONArray jsonArray_lo_wts = (JSONArray) jsonObject_lo.get("wts");      
                JSONArray jsonArray_la_wts = (JSONArray) jsonObject_la.get("wts");
                JSONArray jsonArray_ed_wts = (JSONArray) jsonObject_ed.get("wts");
                JSONArray jsonArray_lo_ta = (JSONArray) jsonObject_lo.get("ta");      
                JSONArray jsonArray_la_ta = (JSONArray) jsonObject_la.get("ta");
                JSONArray jsonArray_ed_ta = (JSONArray) jsonObject_ed.get("ta");

                int len_node_wts = jsonArray_lo_wts.size();
                int len_edge_wts = jsonArray_ed_wts.size();
                int len_node_ta = jsonArray_lo_ta.size();
                int len_edge_ta = jsonArray_ed_ta.size();
                System.out.println("===== System Size =====");
                System.out.println("State in WTS: "+len_node_wts);
                System.out.println("Edge in WTS:  "+len_edge_wts);
                System.out.println("State in TA:  "+len_node_ta);
                System.out.println("Edge in TA:   "+len_edge_ta);

              
                String[] location_w= new String[len_node_wts];
                String[] edge_w= new String[len_edge_wts];

                String[] location_t= new String[len_node_ta];
                String[] edge_t= new String[len_edge_ta];


                String[][] location_wts= new String[len_node_wts][4];
                String[][] edge_wts= new String[len_edge_wts][5];
                String[] label_wts= new String[len_node_wts];

                String[][] location_ta= new String[len_node_ta][4];
                String[][] edge_ta= new String[len_edge_ta][5];
                String[] label_ta= new String[len_node_ta];
                


                for (int i=0; i<len_node_wts; i++){
                    location_w[i] = jsonArray_lo_wts.get(i).toString(); 
                    String la = jsonArray_la_wts.get(i).toString().replace("[", "").replace("]", "").replace("\"", "");
                    label_wts[i] = "null".equals(la.replace("\"", "")) ? null :  la.replace("\"", "");
                }
                for (int i=0; i<len_edge_wts; i++){
                    edge_w[i] = jsonArray_ed_wts.get(i).toString(); 
                }

                for (int i=0; i<len_node_ta; i++){
                    location_t[i] = jsonArray_lo_ta.get(i).toString(); 
                    String la = jsonArray_la_ta.get(i).toString().replace("[", "").replace("]", "").replace("\"", "");
                    label_ta[i] = "null".equals(la.replace("\"", "")) ? null :  la.replace("\"", "");
                }  
                for (int i=0; i<len_edge_ta; i++){
                    edge_t[i] = jsonArray_ed_ta.get(i).toString();
                }    
                

                for (int i=0; i<len_node_wts; i++){
                    String[] lo_sp = location_w[i].replace("[", "").replace("]", "").split(",");
                    String checkinit = lo_sp[4].replace("\"", "");
                    if(checkinit.equals("true")){
                        // System.out.println("[4] is here"+checkinit.getClass().getName());
                        inital_state[0] = i;
                    }
                    for (int j=0; j<4; j++){
                        location_wts[i][j] = "null".equals(lo_sp[j].replace("\"", "")) ? null :  lo_sp[j].replace("\"", "");
                    }
                }
                for (int i=0; i<len_edge_wts; i++){
                    String[] ed_sp = edge_w[i].replace("[", "").replace("]", "").split(",");
                    for (int j=0; j<5; j++){
                        edge_wts[i][j] =  "null".equals(ed_sp[j].replace("\"", "")) ? null :  ed_sp[j].replace("\"", "");
                    }
                }

   
                for (int i=0; i<len_node_ta; i++){
                    String[] lo_sp = location_t[i].replace("[", "").replace("]", "").split(",");
                    if(lo_sp[4] == "true"){
                        inital_state[1] = i;
                    }
                    for (int j=0; j<4; j++){
                        location_ta[i][j] = "null".equals(lo_sp[j].replace("\"", "")) ? null :  lo_sp[j].replace("\"", "");
                        //System.out.println("the location_ta is " + location_ta[i][j]);
                    }
                }
                for (int i=0; i<len_edge_ta; i++){
                    String[] ed_sp = edge_t[i].replace("[", "").replace("]", "").split(",");
                    for (int j=0; j<5; j++){
                        edge_ta[i][j] = "null".equals(ed_sp[j].replace("\"", "")) ? null :  ed_sp[j].replace("\"", "").replace("~", ",");
                        //System.out.println("the edge_ta is " + edge_ta[i][j]);
                    }
                }
                System.out.println("===== Initial State =====");
                System.out.println("Initial state in WTS: "+inital_state[0]);
                System.out.println("Initial state in TA: "+inital_state[1]);

                String[][][] locations = {location_wts, location_ta};
                String[][] labels = {label_wts, label_ta};
                String[][][] edges = {edge_wts,edge_ta};

               
				doc = createSampleModel(docsystem, docdecl, templist, tempdecl, locations, labels, edges, inital_state);
			} else {
				// load model from a file/internet:
				doc = loadModel(args[0]);
			}
            // save the model into a file:
			doc.save(direction + "/"+strDate +"/output/"+"MAS_SR" + flag + ".xml");
			// load a model from the file:
			doc = loadModel(direction + "/"+strDate +"/output/"+"MAS_SR" + flag + ".xml");

			// connect to the engine server:
			Engine engine = connectToEngine();

            // compile the document into system representation:
			UppaalSystem sys = compile(engine, doc);

			// perform a random symbolic simulation and get a trace:
            // System.out.println("===== Random symbolic simulation =====");
			strace = symbolicSimulation(engine, sys);
			// save the trace to an XTR file:
            // System.out.println("===== Trace saving and loading =====");
			saveXTRFile(strace, direction + "/"+strDate +"/output/"+"MAS_SR" + flag + ".xtr");
			Parser parser = new Parser(new FileInputStream(direction + "/"+strDate +"/output/"+"MAS_SR" + flag + ".xtr"));
			// parse a trace from an XTR file:
			// strace = parser.parseXTRTrace(sys);
			// printTrace(sys, strace);


            if (declaration2verify != null) {

                //get the verification from String docdecl
                String search  = "bool";
                String[] words = declaration2verify.split("bool");
                String verification = words[1].trim().replace(",","&&").replace(";", "");
                //System.out.println("The verification is: "+verification+"\n");
                String evenVeri = "E<>("+verification+")";

                // simple model-checking:
                Query query = new Query(evenVeri, "can the product WTS finish?");
                System.out.println("===== Symbolic Verification: "+query.getFormula()+" =====");
                strace = null;
                //QueryResult result = engine.query(sys, options, query, qf);
                //System.out.println("Result: " + result);


                // test thread
                System.out.println("create FooVeri");
                FooVeri fooveri = new FooVeri(engine, sys, options, query, qf);

                System.out.println("create thread");
                Thread thread = new Thread(fooveri);

                System.out.println("starting thread");
                thread.start();

                TimeUnit.SECONDS.sleep(5);
                System.out.println("getvalue");
                QueryResult result = fooveri.getValue();
                
                System.out.println("Result from thread: " + result);
                thread.interrupt();
                printTrace(sys, strace);

                if (strace == null) {
                    //System.out.println("I need to saving an empty path");
                    FileWriter writer = new FileWriter(direction + "/"+strDate +"/output/"+"pathoutput" + flag + ".txt"); 
                    //System.out.println("I am saving an empty path");

                    System.out.println("Java time " + LocalDateTime.now());
                    writer.write("");
                    writer.close();
                    engine.cancel();
                    //engine.disconnect();
                }
                else{
                    List<String> path_wts = new ArrayList<String>();
                    List<String> path_ta = new ArrayList<String>();

                    Iterator<SymbolicTransition> it = strace.iterator();
                    while (it.hasNext()){
                        int p = 0;
                        for (SystemLocation l: it.next().getTarget().getLocations()) {
                            // System.out.println("p is "+p+ " name is " + l.getName());
                            if (p == 0){
                                path_wts.add(l.getName());
                            } 
                            else {
                                path_ta.add(l.getName());
                            }
                            p++;
                        }
                    }
                    FileWriter writer = new FileWriter(direction + "/"+strDate +"/output/"+"pathoutput" + flag + ".txt"); 
                    for(String str: path_wts) {
                    writer.write(str + System.lineSeparator());
                    }
                    writer.close();
                }
                
            }
            else{
                System.out.println("Nothing to verify! ");
            }
            
			engine.disconnect();

        } catch (CannotEvaluateException | EngineException | IOException ex) {
			System.out.flush();
            ex.printStackTrace(System.err);
            System.exit(1);
        }
    }



    public static final String options = "--search-order 0 --diagnostic 0";
    // see "verifyta --help" for the description of options

    public static QueryFeedback qf =
		new QueryFeedback() {
			@Override
			public void setProgressAvail(boolean availability)
			{
			}

			@Override
			public void setProgress(int load, long vm, long rss, long cached, long avail, long swap, long swapfree, long user, long sys, long timestamp)
			{
			}

			@Override
			public void setSystemInfo(long vmsize, long physsize, long swapsize)
			{
			}

			@Override
			public void setLength(int length)
			{
			}

			@Override
			public void setCurrent(int pos)
			{
			}

			@Override
			public void setTrace(char result, String feedback,
								 SymbolicTrace trace, QueryResult queryVerificationResult)
			{
				strace = trace;
			}

			public void setTrace(char result, String feedback,
								 ConcreteTrace trace, QueryResult queryVerificationResult)
			{
				ctrace = trace;
			}
			@Override
			public void setFeedback(String feedback)
			{
				if (feedback != null && feedback.length() > 0) {
					System.out.println("Feedback: "+feedback);
				}
			}

			@Override
			public void appendText(String s)
			{
				if (s != null && s.length() > 0) {
					System.out.println("Append: "+s);
				}
			}

			@Override
			public void setResultText(String s)
			{
				if (s != null && s.length() > 0) {
					System.out.println("Result: "+s);
				}
			}
		};
}

class FooVeri implements Runnable {
    private volatile QueryResult result;
    private volatile Engine engine;
    private volatile UppaalSystem sys; 
    private volatile String options;
    private volatile Query query; 
    private volatile QueryFeedback qf;


    public FooVeri(Engine engine, UppaalSystem sys,  String options, Query query, QueryFeedback qf) {
        // store parameter for later user
     
        this.engine = engine;
        this.sys = sys;
        this.options = options;
        this.query = query;
        this.qf = qf;
    }

    @Override
    public void run() {
        try{
            //TimeUnit.SECONDS.sleep(15);
            this.result = engine.query(sys, options, query, qf);
        }
        catch (Exception ex) {
            System.out.flush();
            ex.printStackTrace(System.err);
            System.exit(1);
        }
    }
    public QueryResult getValue() {
        return result;
    }
}







