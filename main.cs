using System;
using System.Linq;
using System.IO;
using System.Text;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Threading;

internal class WayPointSolver
{
    public WayPointSolver()
    {
    }

    static public Node WpA { get; internal set; }    //Prev waypoint
    static public Node WpB { get; internal set; }    //Next waypoint

    internal void FindNextAndPrev(Vector current, Vector goal)
    {
        //Først må vi finne ut om vi har klar bane til målet
        var H = new Vector(goal.X, 3000);

        Vector Top = null;

        foreach (var T in MarsLander.WaypointCandidates)
        {
            var CG = new Vector(current, goal);
            var GT = new Vector(goal, T);

            //Hvis T er lengre fra målet enn meg, så er det ikke relevant
            if (GT.Length() > CG.Length())
                continue;

            //Hvis T er på andre siden fra målet, er det ikke relevant
            if (Math.Sign(CG.X) == Math.Sign(GT.X))
                continue;

            //Hvis T er målet, er det ikke relevant
            if (T.Minus(goal).Length() < 2)
                continue;

            var GH = new Vector(goal, H);
            var skyIsOnTheLeft = Vector.Cross(CG, GH) > 0;
            var topIsOnTheLeft = Vector.Cross(CG, GT) > 0;

            if (skyIsOnTheLeft == topIsOnTheLeft)
            {
                //Might get a collision here, we should go to T instead
                Top = T;
            }
        }

        if (Top == null)
        {
            Mars.DebugMessage("DirectToGoal");
            DirectToGoal(current, goal);
        }
        else
        {
            Mars.DebugMessage("ToTopPoint");
            ToTopPoint(current, goal, Top);
        }
    }

    private static void ToTopPoint(Vector current, Vector _goal, Vector TopVector)
    {
        var Top = new Vector(TopVector.X, TopVector.Y);
        Top.Y += 100;

        Mars.DebugMessage("Instead of going to " + _goal.DebugStr("Goal"));
        Mars.DebugMessage("We'll be going to to " + Top.DebugStr("Goal"));

        //Will be going to a top
        WpA = new Node();
        WpA.pos = new Vector();
        WpA.Speed = new Vector();

        WpB = new Node();
        WpB.pos = new Vector();
        WpB.Speed = new Vector();

        var landingZone = 500;
        var approachZone = 1000;
        var transportZone = 3000;
        var mapStart = 7000;

        NegateIfNecessary(current, Top, ref landingZone, ref approachZone, ref transportZone, ref mapStart);
        double dy = Top.Y - current.Y; double dx = Math.Abs(Top.X - current.X);
        double a = (dy / dx);

        WpA.pos = new Vector(current.X, current.Y);
        WpB.pos = new Vector(Top.X, Top.Y);

        WpA.Speed.X = 90; WpA.Speed.Y = Math.Max(-40, 90 * a);
        WpB.Speed.X = 30; WpB.Speed.Y = Math.Max(-40, 30 * a);
        NegateSpeedIfNecessary(current, Top);
    }

    private static void DirectToGoal(Vector current, Vector goal)
    {
        WpA = new Node();
        WpA.pos = new Vector();
        WpA.Speed = new Vector();

        WpB = new Node();
        WpB.pos = new Vector();
        WpB.Speed = new Vector();

        var distToGoal = goal.Minus(current).Abs();

        var landingZone = 500;
        var approachZone = 1000;
        var transportZone = 3000;
        var mapStart = 7000;

        NegateIfNecessary(current, goal, ref landingZone, ref approachZone, ref transportZone, ref mapStart);
        double dy = goal.Y - current.Y; double dx = Math.Abs(goal.X - current.X);
        double a = (dy / dx);

        if (distToGoal.X > Math.Abs(transportZone) + 500)
        {
            WpA.pos = current;
            WpB.pos = new Vector(goal.X + transportZone, current.Y + a * (transportZone - approachZone));

            WpA.Speed.X = 80; WpA.Speed.Y = Math.Max(-40, 80 * a);
            WpB.Speed.X = 70; WpB.Speed.Y = Math.Max(-40, 70 * a);
        }
        else if (distToGoal.X > Math.Abs(approachZone) + 500)
        {
            WpA.pos = current;
            WpB.pos = new Vector(goal.X + approachZone, current.Y + (a) * (approachZone - landingZone));

            WpA.Speed.X = 70; WpA.Speed.Y = Math.Max(-40, 70 * a);
            WpB.Speed.X = 35; WpB.Speed.Y = Math.Max(-40, 35 * a);
        }

        else if (distToGoal.X > Math.Abs(landingZone) + 500)
        {
            WpA.pos = current;
            WpB.pos = new Vector(goal.X + landingZone, goal.Y + (a) * landingZone);

            WpA.Speed.X = 35; WpA.Speed.Y = Math.Max(-40, 35 * a);
            WpB.Speed.X = 30; WpB.Speed.Y = Math.Max(-40, 20 * a);
        }
        else
        {
            WpA.pos = current;
            WpB.pos = new Vector(goal.X, goal.Y);

            WpA.Speed.X = 30; WpA.Speed.Y = Math.Max(-40, 40 * a);  //Øker til 30, fordi den hadde problemer med komme seg til midten.
            WpB.Speed.X = 0; WpB.Speed.Y = Math.Max(-30, 30 * a);
        }
        NegateSpeedIfNecessary(current, goal);
    }

    private static void NegateSpeedIfNecessary(Vector current, Vector goal)
    {
        if (current.X > goal.X)
        {
            WpA.Speed.X = -WpA.Speed.X;
            WpB.Speed.X = -WpB.Speed.X;
        }
    }

    private static void NegateIfNecessary(Vector current, Vector goal, ref int landingZone, ref int approachZone, ref int transportZone, ref int mapStart)
    {
        bool weAreLeftOfGoal = current.X < goal.X;
        if (weAreLeftOfGoal)
        {
            transportZone = -transportZone;
            approachZone = -approachZone;
            landingZone = -landingZone;
            mapStart = -mapStart;
        }
    }
}

[DebuggerDisplay("({X},{Y})")]
public class Vector
{
    public double X;
    public double Y;

    public Vector(double xx, double yy)
    {
        X = xx; Y = yy;
    }
    public Vector() { }

    public Vector(Vector A, Vector B)
    {
        X = B.X - A.X;
        Y = B.Y - A.Y;
    }

    public override string ToString()
    {
        return $"({X},{Y})";
    }

    public bool Equals(Vector B)
    {
        var length = Distance(B);
        return length < 2;
    }

    public void Debug(string msg)
    {
        Console.Error.Write(DebugStr(msg));
    }

    public static Vector Minus(Vector A, Vector B)
    {
        return new Vector(A.X - B.X, A.Y - B.Y);
    }

    public static Vector Mult(Vector A, double s)
    {
        return new Vector(A.X * s, A.Y * s);
    }

    public Vector Mult(double s)
    {
        return new Vector(X * s, Y * s);
    }

    public static Vector Plus(Vector A, Vector B)
    {
        return new Vector(A.X + B.X, A.Y + B.Y);
    }

    public double Length()
    {
        return Math.Sqrt(X * X + Y * Y);
    }

    public Vector Unit()
    {
        var length = Length();
        return new Vector(X / length, Y / length);
    }
    //sign((b-a).cross(m-a)[2])    public static bool IsOnTheLeftSide(Vector A, Vector B, Vector M)
    {
        Vector AB = new Vector(A, B);
        Vector AM = new Vector(A, M);

        double cross = Cross(AB, AM);

        return cross > 0;
    }

    public static double Cross(Vector A, Vector B)
    {
        return (A.X * B.Y) - (A.Y * B.X);
    }

    internal string DebugStr(string msg)
    {
        return $"{msg} ({X:F0},{Y:F0})";
    }

    internal double Distance(Vector nextWaypoint)
    {
        var dist = Vector.Minus(this, nextWaypoint).Length();
        return dist;
    }

    internal Vector Floor()
    {
        return new Vector((int)X, (int)Y);
    }

    internal Vector Abs()
    {
        return new Vector(Math.Abs(X), Math.Abs(Y));
    }

    internal Vector Minus(Vector B)
    {
        return Vector.Minus(this, B);
    }

    internal double Dot(Vector speed)
    {
        return this.X * speed.X + this.Y * speed.Y;
    }

    internal Vector Round()
    {
        return new Vector(Math.Round(this.X), Math.Round(this.Y));
    }
}

public class Action
{
    public int rotation;
    public int power;
}


class GFG
{
    public static double distanceFromPointToLineSegment(Vector pt, Vector x1y1, Vector x2y2)
    {
        double x = pt.X;
        double y = pt.Y;
        double x1 = x1y1.X;
        double y1 = x1y1.Y;
        double x2 = x2y2.X;
        double y2 = x2y2.Y;

        var A = x - x1;
        var B = y - y1;
        var C = x2 - x1;
        var D = y2 - y1;

        var dot = A * C + B * D;
        var len_sq = C * C + D * D;
        var param = -1.0;
        if (len_sq != 0) //in case of 0 length line
            param = dot / len_sq;

        double xx, yy;

        if (param < 0)
        {
            xx = x1;
            yy = y1;
        }
        else if (param > 1)
        {
            xx = x2;
            yy = y2;
        }
        else
        {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        var dx = x - xx;
        var dy = y - yy;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    // Given three colinear points p, q, r, the function checks if
    // point q lies on line segment 'pr'
    static Boolean onSegment(Vector p, Vector q, Vector r)
    {
        if (q.X <= Math.Max(p.X, r.X) && q.X >= Math.Min(p.X, r.X) &&
            q.Y <= Math.Max(p.Y, r.Y) && q.Y >= Math.Min(p.Y, r.Y))
            return true;

        return false;
    }

    // To find orientation of ordered triplet (p, q, r).
    // The function returns following values
    // 0 --> p, q and r are colinear
    // 1 --> Clockwise
    // 2 --> Counterclockwise
    static int orientation(Vector p, Vector q, Vector r)
    {
        // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
        // for details of below formula.
        double val = (q.Y - p.Y) * (r.X - q.X) -
                (q.X - p.X) * (r.Y - q.Y);

        if (val == 0) return 0; // colinear

        return (val > 0) ? 1 : 2; // clock or counterclock wise
    }

    // The main function that returns true if line segment 'p1q1'
    // and 'p2q2' intersect.
    public static Boolean doIntersect(Vector p1, Vector q1, Vector p2, Vector q2)
    {
        // Find the four orientations needed for general and
        // special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;

        // p1, q1 and q2 are colinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;

        // p2, q2 and p1 are colinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;

        // p2, q2 and q1 are colinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false; // Doesn't fall in any of the above cases
    }
}

[DebuggerDisplay("Visits {Visits} Win: {Win}")]
public class Stats
{
    public double Win { get; set; }
    public int Visits { get; set; }

    public override string ToString()
    {
        return $"win {Win:F} visits {Visits}";
    }
}

//[DebuggerDisplay("Id: {Id} F:{F}, pos:{Pos} speed:{Speed} fuel{fuel} rotate{rotate} power{power}")]
[DebuggerDisplay("{ToString()}")]
public class Node
{
    const double mcts_constant = 1.41;
    public static int nextId = 0;
    public List<Node> children;
    protected bool sim_finished = false;
    public Node parent;
    public Stats stats { get; set; }
    public bool Visited { get; set; }
    public bool opened { get; set; }
    public int depth { get; set; }
    private int Id;
    public Vector pos;
    public Vector Speed;
    public int fuel;
    public int rotate;
    public int power;

    public Vector Targetspeed;

    public double Diff { get; private set; }

    public double F; //Cost Distance for waypoint

    public Node()
    {
        this.children = new List<Node>();
        this.stats = new Stats();
        this.Id = Node.nextId++;

    }

    public Node(Node from)
        : this()
    {
        this.depth = from.depth + 1;

        this.pos = from.pos;
        this.Speed = from.Speed;
        this.fuel = from.fuel;
        this.rotate = from.rotate;
        this.power = from.power;
        this.parent = from;
    }

    //public virtual void open() { throw new NotImplementedException(); }
    public virtual void do_sim()
    {
        var dV = new Vector(
            -Math.Sin(this.rotate / 180.0 * Math.PI) * this.power,
            Math.Cos(this.rotate / 180.0 * Math.PI) * this.power - 3.711);

        var dV2 = dV.Mult(0.5);

        var SpeedAverage = Vector.Plus(this.Speed, dV2);
        this.Speed = Vector.Plus(this.Speed, dV);

        this.pos = Vector.Plus(this.pos, SpeedAverage);
        this.fuel -= this.power;

        var targetSpeed = this.interpolate(this.pos);
        var speedLength = this.Speed.Length();
        var targetSpeedLength = targetSpeed.Length();

        if (speedLength > targetSpeedLength * 1.2)
        {
            targetSpeed = new Vector(targetSpeed.X > 0 ? 20 : -10, 0);
        }

        this.Targetspeed = targetSpeed;
        var diff = this.Speed.Minus(targetSpeed).Abs();
        var diffPercent = diff.Mult(1 / targetSpeed.Length());
        this.FArray[2] = 1.0 - diffPercent.X - diffPercent.Y * 1.5;
        this.Diff = this.FArray[1] + this.FArray[2] + this.FArray[3];
        this.F = this.Diff - Node.RootDiff;
    }



    //Må tenke på om vi skal bruke X eller ikke når vi beregner prosent for y, siden det har jo litt å si om vi har kommet nærme i 
    //x for y-hastigheten. Samme med x, men his man er veldig nærme i x, så ødelegerr en verdi langt unna i y, 
    //da hopper target-hastighet fram og tilbake

    private Vector interpolate(Vector pos)
    {

        //compute the desired speed, or absolute value of it, given the desired speeds and positions of A and B
        var posA = WayPointSolver.WpA.pos;
        var posB = WayPointSolver.WpB.pos;

        var desiredSpeedA = WayPointSolver.WpA.Speed;
        var desiredSpeedB = WayPointSolver.WpB.Speed;

        var legDist = posB.Minus(posA).Length();
        var distLeft = posB.Minus(pos).Abs();
        var percentLeft = distLeft.Mult(1.0 / legDist);

        //kan ikke gjøre det på vektorform, for man kan være veldig nærme i X, og langt unna i Y,
        //Da vil en felles prosentdel kunne gi store svingninger for den verdien som er nærme
        var speedA = new Vector();
        speedA.X = desiredSpeedA.X * percentLeft.X; //Hvis vi har percentLeft=0.95, har nesten ikke startet, skal vi ha nesten kun speedA
        //speedA.Y = desiredSpeedA.Y*percentLeft.Y;
        speedA.Y = desiredSpeedA.Y * percentLeft.X;

        var speedB = new Vector();
        speedB.X = desiredSpeedB.X * (1 - percentLeft.X);
        //speedB.Y = desiredSpeedB.Y * (1-percentLeft.Y);
        speedB.Y = desiredSpeedB.Y * (1 - percentLeft.X);             //Hvor langt vi har kommet i X er viktigst per nå

        var targetSpeed = Vector.Plus(speedA, speedB);
        return targetSpeed;
    }

    private static double FindClosestSurface(Node child)
    {
        double closest = 1000000;
        for (int surfaceIndex = 0; surfaceIndex < MarsLander.Terrain.Count - 2; surfaceIndex++)
        {
            var A = MarsLander.Terrain[surfaceIndex];
            var B = MarsLander.Terrain[surfaceIndex + 1];

            double dist = GFG.distanceFromPointToLineSegment(child.pos, A, B);
            if (dist < closest)
            {
                closest = dist;
            }
        }

        return closest;
    }

    public virtual double result()
    {
        //var f = 1 + (1 / this.F) * 1000;
        var f = this.F;

        //Bør ikke ha så veldig forskjellig fra -1 og 1

        return f;
    }
    public virtual void Act(IReader reader, Node bestNode, int iteration)
    {
        var action = bestNode.GetAction();
        var rotation = action.Item1;
        var outputPower = action.Item2;

        var dist = bestNode.pos.Distance(MarsLander.Goal);
        if (//dist < 500 && 
            Math.Abs(bestNode.pos.Y - MarsLander.Goal.Y) < 50)
        {
            Mars.DebugMessage("Less than 50 above, stop rotation");
            rotation = 0;
        }

        //if (iteration < 8 && Math.Abs(rotation) > 45)
        //{
        //    rotation = 0;
        //}

        if (iteration < 15 && Math.Abs(rotation) >= 45)
        {
            if (rotation >= 45)
                rotation = 45;
            if (rotation <= -45)
                rotation = -45;
        }

        if (iteration < 10)
        {
            outputPower = 4;
        }

        //Pga den som kommer inn med stor fart i starten, og står i fare for å krasje i fjellside på sdien
        if (iteration < 8 && Math.Abs(bestNode.Speed.X - bestNode.Targetspeed.X) > 50)
        {
            rotation = 0;
            outputPower = 4;
        }


        //Hvis vi er langt unna, og retningen er riktig, kjør på i 100
        if (iteration < 5 && Math.Abs(bestNode.pos.X - MarsLander.Goal.X) > 3000)
        {
            Mars.DebugMessage("Langt unna, forsøk å gasse på hvis vinkelen er riktig");

            bool correctAngle = rotation > 0 && bestNode.pos.X > MarsLander.Goal.X
                || rotation < 0 && bestNode.pos.X < MarsLander.Goal.X;
            if (correctAngle)
            {
                Mars.DebugMessage("Vi har riktig vinkel for å gasse på");
                outputPower = 4;
            }
        }

        Mars.DebugMessage($"Distance {dist}");

        Console.WriteLine($"{rotation} {outputPower}");
        Debug.WriteLine($"{rotation} {outputPower}");

        //Change the node, so it will give the correct simulation line


        var parent = bestNode.parent;
        var changedNode = new Node(bestNode.parent);

        var parentRot = parent.rotate;
        var parentPower = parent.power;

        if (outputPower > parentPower + 1)
        {
            changedNode.power = parentPower + 1;
        }
        if (outputPower < parentPower - 1)
        {
            changedNode.power = parentPower - 1;
        }

        if (rotation > parentRot + 15)
        {
            changedNode.rotate = parentRot + 15;
        }

        if (rotation < parentRot - 15)
        {
            changedNode.rotate = parentRot - 15;
        }
        changedNode.do_sim();

        bestNode.pos = changedNode.pos;
        bestNode.Speed = changedNode.Speed;
        bestNode.rotate = changedNode.rotate;
        bestNode.power = changedNode.power;
        bestNode.fuel = changedNode.fuel;
    }
    public double uct_value()
    {
        if (parent == null)
            return 1000000; //start node has noparent

        //Usikker på om vi skal ha ln, log10 el log2 her
        //double log_parent_visits = Math.Log10(parent.stats.Visits);
        double log_parent_visits = Math.Log(parent.stats.Visits);
        if (stats.Visits == 0)
            return 1000000;

        var uct = stats.Win / stats.Visits + mcts_constant * Math.Sqrt(2 * log_parent_visits / stats.Visits);
        return uct;
    }

    public virtual Node pick_random()
    {
        int size = this.children.Count();
        int index = MarsLander.rnd.Next(size);
        var picked = this.children[index];
        return picked;
    }

    //private static int nextId = 0;


    public override string ToString()
    {
        return $"depth {depth} stats {stats} opened {opened} Id: {Id} ({pos.X:F0},{pos.Y:F0}) ({Speed?.X:F1} {Speed?.Y:F1}) (r{rotate},p{power}) F {this.F:F4} F1 {this.FArray[1]:F4} F2 {this.FArray[2]:F4} F3 {this.FArray[3]:F4} F4 {this.FArray[4]:F4} F5 {this.FArray[5]:F4} F6 {this.FArray[6]:F4} F7 {this.FArray[7]:F4} Danger {this.CloseDanger}";
    }

    public double CloseDanger { get; private set; }
    public static double RootDiff { get; set; }

    public double[] FArray = new double[8];
    //public double F2 { get; private set; }
    //public double F3 { get; private set; }
    //public double F4 { get; private set; }
    //public double F5 { get; private set; }
    //public double F6 { get; private set; }
    //public int F7 { get; private set; }

    public bool CanReach(global::Vector From, global::Vector To)
    {
        //Only possible if the terrain is not blocking
        for (int posIndex = 0; posIndex < MarsLander.Terrain.Count() - 2; posIndex++)
        {
            var A = MarsLander.Terrain[posIndex];
            var B = MarsLander.Terrain[posIndex + 1];

            if (GFG.doIntersect(A, B, From, To))
            {
                return false;
            }
        }

        return true;
    }

    internal List<Node> GetNeighbours(List<Node> Graph)
    {
        var n = new List<Node>();
        foreach (var candidate in Graph)
        {
            if (candidate == this)
                continue;

            if (CanReach(this.pos, candidate.pos))
            {
                //Mars.DebugMessage($"{this.Pos} can reach {candidate.Pos}");
                n.Add(candidate);
            }
        }
        return n;
    }

    //Egentlig bør wp være felt i node

    internal void open()
    {
        if (this.opened)
        {
            Mars.DebugMessage("Should not go here twice");
        }


        this.opened = true;
        var candidateChildren = new List<Node>();

        for (int rot = -15; rot <= 15; rot += 15)
        {
            for (int pow = -1; pow <= 1; pow++)
            {

                var newRot = this.rotate + rot;
                var newPow = this.power + pow;

                //TODO, sjekk om vi egentlig trenger å teste på 90 og -90. Tror ikke de er mye i bruk
                //Sjekk -90/+90 til slutt, og bare legg dem til om vi ikke har noen bedre

                //if (newRot <= 75 && newRot > -75 && newPow >= 0 && newPow <= 4)//Var noen som ikke fikk noen barn
                if (newRot <= 90 && newRot > -90 && newPow >= 0 && newPow <= 4)
                {
                    var child = new Node(this);
                    candidateChildren.Add(child);
                    child.power = newPow;
                    child.rotate = newRot;
                    children.Add(child);
                }
            }
        }
    }

    public (int, int) GetAction()
    {
        return (rotate, power);
    }

    public static int linesPrinted = 0;

    internal void PrintChildren(int currentLevel, int maxLevel)
    {
        if (linesPrinted++ < 100)
        {
            var indent = new string(' ', currentLevel * 3);
            var line = indent + ToString();
            Mars.DebugMessage(line);

            if (maxLevel > currentLevel)
            {
                foreach (var child in children)
                {
                    child.PrintChildren(currentLevel + 1, maxLevel);
                }
            }
        }
    }
}

class DjikstreaSearch
{
    //     1  function Dijkstra(Graph, source):
    // 2
    // 3      create vertex set Q
    // 4
    // 5      for each vertex v in Graph:            
    // 6          dist[v] ← INFINITY                 
    // 7          prev[v] ← UNDEFINED                
    // 8          add v to Q                     
    // 9      dist[source] ← 0                       
    //10     
    //11      while Q is not empty:
    //12          u ← vertex in Q with min dist[u]   
    //13                                             
    //14          remove u from Q
    //15         
    //16          for each neighbor v of u:           // only v that are still in Q
    //17              alt ← dist[u] + length(u, v)
    //18              if alt<dist[v]:              
    //19                  dist[v] ← alt
    //20                  prev[v] ← u
    //21
    //22      return dist[], prev[]

    public Node GetShortestPath(Node start, global::Vector Goal)
    {
        Node goalNode = null;

        var Q = new List<Node>();
        var Graph = new List<Node>();
        Q.Add(start);
        Graph.Add(start);
        start.F = 0;

        foreach (var pos in MarsLander.WaypointCandidates)
        {
            var v = new Node();
            v.pos = pos;

            v.F = 1000000000;
            Q.Add(v);
            Graph.Add(v);

            if (Goal.Equals(pos))
            {
                goalNode = v;
            }
        }

        for (var u = Q.OrderBy(x => x.F).FirstOrDefault(); u != null; u = Q.OrderBy(x => x.F).FirstOrDefault())
        {
            Q.Remove(u);

            List<Node> neighbours = u.GetNeighbours(Graph);
            foreach (var v in neighbours)
            {
                var alt = u.F + u.pos.Distance(v.pos);
                if (alt < v.F)
                {
                    v.F = alt;
                    v.parent = u;
                }
            }
        }
        return goalNode;
    }

}


public class MonteCarloTreeSearch
{
    //Pseudocodde
    //def monte_carlo_tree_search(root) :
    //	while resources_left(time, computational power) :
    //      leaf = traverse(root) # leaf = unvisited node
    //	    simulation_result = rollout(leaf)
    //      backpropagate(leaf, simulation_result)
    //	return best_child(root)

    //# function for node traversal
    //def traverse(node) :
    //	while fully_expanded(node) :
    //     node = best_uct(node)

    //	# in case no children are present / node is terminal
    //	return pick_univisted(node.children) or node

    //def rollout(node):
    //	while non_terminal(node) :
    //    node = rollout_policy(node)
    //	return result(node)

    //def rollout_policy(node) :
    //	return pick_random(node.children)

    //def backpropagate(node, result) :
    //	if is_root(node)
    //		return
    //	node.stats = update_stats(node, result)
    //  backpropagate(node.parent) <--- her er det feil, mangler argument

    //def best_child(node) :
    //    pick child with highest number of visits

    private Stopwatch stopwatch;
    private int rolloutsProcessed;
    public bool UseTimer { get; set; }
    public int MaxDepth { get; set; }
    public int MaxRollouts { get; set; }


    bool resources_left()
    {
        this.rolloutsProcessed++;

        if (this.UseTimer)
        {
            var diff = this.stopwatch.ElapsedMilliseconds;  //TODO CPU-tid, ikke sanntid
            return diff < 50;
        }
        else
        {
            return this.rolloutsProcessed < this.MaxRollouts;
        }
    }

    Node monte_carlo_tree_search(Node root)
    {
        this.stopwatch = new Stopwatch();
        this.stopwatch.Start();
        this.rolloutsProcessed = 0;

        while (resources_left())
        {
            var leaf = traverse(root);
            var simulation_result = rollout(leaf);
            backpropagate(leaf, simulation_result);
        }

        Mars.DebugMessage($"Rollouts processed: {this.rolloutsProcessed}");
        return best_child(root);
    }

    Node traverse(Node node)
    {
        while (fully_expanded(node))
        {
            node = best_uct(node);

            if (node.depth >= this.MaxDepth) //Jeg har lagt til denne
                return node;
        }

        var child = pick_unvisited(node); // or node
        if (child == null)
            return node;

        return child;
    }


    Node pick_unvisited(Node node)
    {
        Node result = null;

        if (!node.opened)
        {
            node.open();
            //node.opened = true;
        }
        foreach (var child in node.children)
        {
            if (!child.Visited)
            {
                child.do_sim();
                //Ikke blande sammen Visited og Visits. Selv om Visited = Visits > 0 
                child.Visited = true;
                //return child as Node;  //kommentert ut 27.07 for debugging
                result = child;
                break;
            }
        }

        node.children = node.children.OrderByDescending(n => n.F).ToList(); //Mars lander, for bedre oversikt
        return result;
    }

    double rollout(Node node)
    {
        while (!is_terminal(node))
        {
            node = rollout_policy(node);
            node.do_sim();
        }
        var res = node.result();
        return res;
    }

    bool is_terminal(Node node)
    {
        if (node.depth >= this.MaxDepth)
            return true;

        if (node.pos.Y < 0)
            return true;

        return false;
    }

    Node rollout_policy(Node node)
    {
        return pick_random(node);
    }



    public Node pick_random(Node node)
    {
        if (!node.opened) //ble lagt til på marslander. 
            node.open(); //ble lagt til på marslander. 


        var picked = node.pick_random();
        picked.do_sim();
        return picked;
    }

    void backpropagate(Node node, double result)
    {
        if (is_root(node))
        {
            node.stats = update_stats(node, result);
            return;
        }
        node.stats = update_stats(node, result);
        var parent = node.parent;
        backpropagate(parent, result);
    }

    bool is_root(Node node)
    {
        return node.parent == null;
    }

    Stats update_stats(Node node, double result)
    {
        node.stats.Visits += 1;
        node.stats.Win += result;

        return node.stats;
    }

    Node best_child(Node node)
    {
        int best_so_far = -1000000;
        Node favourite_child = null;
        foreach (var child in node.children)
        {
            if (child.stats.Visits > best_so_far)
            {
                best_so_far = child.stats.Visits;
                favourite_child = child;
            }
        }
        return favourite_child;
    }
    Node best_uct(Node node)
    {
        Node best_ucts_child_so_far = null;

        double best_ucts_so_far = best_ucts_so_far = -1000000;

        foreach (var child in node.children)
        {
            var ucts = child.uct_value();
            if (ucts > best_ucts_so_far)
            {
                best_ucts_child_so_far = child;
                best_ucts_so_far = ucts;
            }
        }

        return best_ucts_child_so_far;
    }


    bool fully_expanded(Node node)
    {
        if (!node.opened)
            return false;

        if (node.children.Count() == 0)
            return true;

        foreach (var child in node.children)
        {
            if (!child.Visited)
                return false;
        }
        return true;
    }

    public Node Solve(IGameInterface game)
    {
        var startNode = game.GetStartNode();
        Node bestChild = monte_carlo_tree_search(startNode);
        //Program.Debug("States created : " + CodersStrikeBack.CodersStrikeBackState.how_many_states());

        return bestChild;
    }
}//MonteCarloTreeSearch

public interface IReader
{
    string ReadLine();
    void WriteLine(string output);
    void AddLine(string line);
}

public class SimulationReader : IReader
{
    List<string> lines = new List<string>();

    public string ReadLine()
    {
        string line = lines[0];
        Mars.DebugMessage(line);
        lines.RemoveAt(0);
        return line;
    }

    public void AddLine(string line)
    {
        lines.Add(line);
    }

    public void WriteLine(string output)
    {

    }
}

class ConsoleReader : IReader
{
    public void AddLine(string line)
    {

    }

    public string ReadLine()
    {
        var line = Console.ReadLine();
        Mars.DebugMessage(line);
        return line;
    }

    public void WriteLine(string output)
    {
        Console.WriteLine(output);
    }
}

public interface IGameInterface
{
    Node GetStartNode();
}

public class TerrainData
{
    public string Data;
    public string Name;
    public bool landedOk;
    public Vector Speed = new Vector();
    public double FuelLeft;

    public List<string> Messages = new List<string>();
}

//Testcaser
public class MarsLander : IGameInterface
{
    public static Vector Goal = new global::Vector();
    private Node start;
    int iteration;
    static public Random rnd = new Random();

    //Skal bli en rett strek, rett mot mål. Ingen fare for kollisjon
    static public string EasyOnTheRight01 = @"7
0 100
1000 500
1500 1500
3000 1000
4000 150
5500 150
6999 800
2500 2700 0 0 550 0 0";

    //Her er det også en rett linje til mål, men siden man har fart i starten
    //er de umulig å fly linja. man kommer med en utgangfart mot venstre, og dermed blir den optimale linja buet, og
    //man får et ganske stort avvik fra linja. Litt usikker på hva som lønner seg her
    static public string InitialSpeedCorrectSide02 = @"10
0 100
1000 500
1500 100
3000 100
3500 500
3700 200
5000 1500
5800 300
6000 1000
6999 2000
5752 2748 -116 -11 578 45 4"; //test
                              //6500 2800 -100 0 600 90 0"; //Original

    //Ligner på den forrige, men man kommer i så stor fart at man står i fare for å krasje i terrenget
    //Blir en bue her også. 
    static public string InitialSpeedWrongSide03 = @"7
0 100
1000 500
1500 1500
3000 1000
4000 150
5500 150
6999 800
6500 2800 -90 0 750 90 0";

    //Her står man i fare for å krasje i et fjell, hvis man kjører rett mot målet,
    //så man trenger et veipunkt på toppen av det fjellet
    //Her kan vi låne litt fra marslander 3
    static public string DeepCanyon04 = @"20
0 1000
300 1500
350 1400
500 2000
800 1800
1000 2500
1200 2100
1500 2400
2000 1000
2200 500
2500 100
2900 800
3000 500
3200 1000
3500 2000
3800 800
4000 200
5000 200
5500 1500
6999 2800
500 2700 100 0 800 -90 0";

    //Burde gå enkelt med en rett linje fra start til mål
    static public string HighGround05 = @"20
0 1000
300 1500
350 1400
500 2100
1500 2100
2000 200
2500 500
2900 300
3000 200
3200 1000
3500 500
3800 800
4000 200
4200 800
4800 600
5000 1200
5500 900
6000 500
6500 300
6999 500
6500 2700 -50 0 1000 90 0"; //Originalt
                            //4634 2127 -115 -48 934 45 3"; //Test

    public static string Data = @"7
0 100
1000 500
1500 1500
3000 1000
4000 150
5500 150
6999 800
4747 1900 -13 -10 164 15 4";

    static MarsLander()
    {
        MarsLander.TerrainDatas = new List<TerrainData>();
        MarsLander.TerrainDatas.Add(new TerrainData { Data = InitialSpeedCorrectSide02, Name = "InitialSpeedCorrectSide02" });
    }


    public static List<Vector> WaypointCandidates = new List<Vector>();
    public static List<TerrainData> TerrainDatas { get; }
    public static string TerrainName { get; internal set; }
    public static List<Vector> Terrain = new List<Vector>();


    public Node GetStartNode()
    {
        return this.start;
    }

    public void init(IReader reader)
    {
        string[] inputs;

        int surfaceN = int.Parse(reader.ReadLine()); // the number of points used to draw the surface of Mars.
        var surfaceList = new List<(int, int)>();
        int startFlatX = 0;
        int startFlatY = 0;

        for (int i = 0; i < surfaceN; i++)
        {
            var line = reader.ReadLine();
            inputs = line.Split(' ');

            //Mars.DebugMessage($"i:{i} " + line);

            int landX = int.Parse(inputs[0]); // X coordinate of a surface point. (0 to 6999)
            int landY = int.Parse(inputs[1]); // Y coordinate of a surface point. By linking all the points together in a sequential fashion, you form the surface of Mars.

            surfaceList.Add((landX, landY));
            MarsLander.Terrain.Add(new global::Vector(landX, landY));

            if (landY == startFlatY)
            {
                if (Math.Abs(landX - startFlatX) >= 1000)
                {
                    Mars.DebugMessage($"Found flat area at ({startFlatX},{startFlatY})-({landX},{landY})");
                    Goal.X = startFlatX + (landX - startFlatX) / 2;
                    Goal.Y = startFlatY + (landY - startFlatY) / 2 + 1;   //Add 1 to avoid goal to intersect with terrain
                    //Goal = startFlatPos.Plus(landPos.Minus(startFlatPos).Div(2.0));
                    Goal.Debug("Goal");
                }
            }
            else
            {
                startFlatX = landX;
                startFlatY = landY;
            }
        }

        Mars.DebugMessage(string.Join(",", surfaceList));

        //find concave vertices
        //They will have the middle point to the left

        for (int surfaceIndex = 0; surfaceIndex < surfaceList.Count - 3; surfaceIndex++)
        {
            var AA = surfaceList[surfaceIndex];
            var BB = surfaceList[surfaceIndex + 2];
            var MM = surfaceList[surfaceIndex + 1];

            var A = new Vector(AA.Item1, AA.Item2);
            var B = new Vector(BB.Item1, BB.Item2);
            var M = new Vector(MM.Item1, MM.Item2);

            if (global::Vector.IsOnTheLeftSide(A, B, M))
            {
                //M.Debug("Convex"); Console.Error.Write(" ");

                var AM = new Vector(A, M);
                var BM = new Vector(B, M);
                //var P = Vector.Plus(M, global::Vector.Plus(AM, BM).Unit().Mult(100));
                var P = M;
                P = P.Floor();
                //P.Debug("Waypoint"); Mars.DebugMessage(" ");

                WaypointCandidates.Add(P);
            }
        }
        WaypointCandidates.Add(Goal);

        //for every vertices that are sharp, find the shortest distance to another line
        //the waypoint candidate is on the middle of this distance
        Mars.DebugMessage("waypointcandidates");
        Mars.DebugMessage(string.Join(",", WaypointCandidates));

    }

    internal void update(IReader reader)
    {
        Mars.DebugMessage($"Iteration {iteration++}");
        var inputs = reader.ReadLine().Split(' ');
        int X = int.Parse(inputs[0]);
        int Y = int.Parse(inputs[1]);
        int hSpeed = int.Parse(inputs[2]); // the horizontal speed (in m/s), can be negative.
        int vSpeed = int.Parse(inputs[3]); // the vertical speed (in m/s), can be negative.
        int fuel = int.Parse(inputs[4]); // the quantity of remaining fuel in liters.
        int rotate = int.Parse(inputs[5]); // the rotation angle in degrees (-90 to 90).
        int power = int.Parse(inputs[6]); // the thrust power (0 to 4).

        var current = new Node();
        current.pos = new Vector(X, Y);
        current.Speed = new Vector(hSpeed, vSpeed);
        current.fuel = fuel;
        current.rotate = rotate;
        current.power = power;

        this.start = current;

        var wps = new WayPointSolver();
        wps.FindNextAndPrev(current.pos, MarsLander.Goal);
    }
}

class Mars
{
    private static bool Landed(SimulationReader reader, Node bestNext)
    {
        var landed = false;
        var okLanding = true;

        if (bestNext.pos.Y < 0)
        {
            var data = MarsLander.TerrainDatas.First(x => x.Name == MarsLander.TerrainName);
            data.FuelLeft = bestNext.fuel;

            landed = true;
            if (bestNext.Speed.Y < -40)
            {
                string str = $"Fell too fast {bestNext.Speed.Y}";
                data.Messages.Add(str);
                Mars.DebugMessage(str);

                okLanding = false;
            }
            if (Math.Abs(bestNext.Speed.X) > 20)
            {
                string str = $"To much horisontal speed {bestNext.Speed.X}";
                data.Messages.Add(str);
                Mars.DebugMessage(str);
                okLanding = false;
            }

            if (Math.Abs(bestNext.rotate) > 45)
            {
                string str = $"To much much rotation in landing {bestNext.rotate}";
                data.Messages.Add(str);
                Mars.DebugMessage(str);
                okLanding = false;
            }

            if (okLanding)
            {
                string Str = "Ok landing";
                data.Messages.Add(Str);
                Mars.DebugMessage(Str);
            }
            Mars.DebugMessage("Fuel left " + bestNext.fuel);
            data.landedOk = okLanding;
            data.Speed = bestNext.Speed;
        }
        return landed;
    }

    public static void DebugMessage(string str)
    {
        Console.Error.WriteLine(str);
        Debug.WriteLine(str);
    }


    static void Main(string[] args)
    {
        var solver = new MonteCarloTreeSearch
        {
            UseTimer = false,
            MaxDepth = 4,
            MaxRollouts = 1500
        };
        var reader = new ConsoleReader();

        var game0 = new MarsLander();
        game0.init(reader);
        List<Node> waypoints = null;

        bool landed = false;
        var iteration = 0;

        //iteration = 10;
        while (!landed)
        {
            iteration++;
            game0.update(reader);
            var startNode = game0.GetStartNode();
            var copy = new Node(startNode);
            copy.do_sim();
            Node.RootDiff = copy.Diff;

            var bestNext = solver.Solve(game0);
            Mars.DebugMessage($"{bestNext.pos.DebugStr("Pos")} {bestNext.Speed.DebugStr("Speed")} {bestNext.Targetspeed.DebugStr("TargetSpeed")}");
            Mars.DebugMessage($"{WayPointSolver.WpA.pos.DebugStr("WpA")} {WayPointSolver.WpB.pos.DebugStr("WpB")} {WayPointSolver.WpA.Speed.DebugStr("WpASpeed")} {WayPointSolver.WpB.Speed.DebugStr("WpBSpeed")}");

            var root = bestNext.parent;
            Node.linesPrinted = 0;
            root.PrintChildren(1, 20);

            bestNext.Act(reader, bestNext, iteration);

            var b = bestNext;
            var culture = CultureInfo.CreateSpecificCulture("en-US");
            Thread.CurrentThread.CurrentCulture = new CultureInfo("en-US");
            string simulationLine = $"{b.pos.X:F0} {b.pos.Y:F0} {b.Speed.X:F0} {b.Speed.Y:F0} {b.fuel:F0} {b.rotate:F0} {b.power:F0}";
            Mars.DebugMessage("Parameters for next turn: " + simulationLine);
    }
}



