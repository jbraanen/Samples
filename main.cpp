#include <iostream>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <string>
#include <algorithm>
#include <numeric>
#include <functional>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cassert>
#include <chrono>
#include <thread>

#if _WIN32
#include <windows.h>
#endif


using namespace std;
int global_timer = 0;
int global_round = 0;
const int checkPointRadius = 500; //Actually, its 600, but we want to make sure they are well inside

class Log {
	static const int max_log = 100000;
	float log_table[max_log];

public:
	Log() {
		for (int i = 1; i < max_log; i++)
		{
			log_table[i] = ::log(i);
		}
	}
	float log(int val) {
		if (val >= max_log - 1)
			val = max_log - 1;
		return log_table[val];
	}
} global_log;

class SineHP
{

public:

	static float Cos(float x) {
		return Sin(M_PI_2 - x);
	}
	static float Sin(float x)
	{
		float sin = 0;
		while (x < -3.14159265f)
			x += 6.28318531f;

		while (x > 3.14159265f)
			x -= 6.28318531f;

		if (x < 0)
		{
			sin = x * (1.27323954f + 0.405284735f * x);

			if (sin < 0)
				sin = sin * (-0.255f * (sin + 1) + 1);
			else
				sin = sin * (0.255f * (sin - 1) + 1);
		}
		else
		{
			sin = x * (1.27323954f - 0.405284735f * x);

			if (sin < 0)
				sin = sin * (-0.255f * (sin + 1) + 1);
			else
				sin = sin * (0.255f * (sin - 1) + 1);
		}

		return sin;
	}
};


//CPU-time NOT wall time
class Stopwatch {
	std::clock_t c_start;
	std::clock_t c_end;
public:
	void Start() { c_start = std::clock(); }
	void Stop() { c_end = std::clock(); }
	int CpuMilliseconds(bool stop = false) {
		if (stop)
			Stop();
		auto diff = (c_end - c_start);
		int time = (int)(1000.0 * diff / CLOCKS_PER_SEC);
		return time;
	}
	void print_time(const char* msg) {
		auto diff = (c_end - c_start);
		cerr << msg << " " << std::fixed << std::setprecision(2) << "cpu time: "
			<< 1000.0 * diff / CLOCKS_PER_SEC << " ms" << endl;
	}
};

Stopwatch global_watch;

class AverageWatch {
	long time;
	long count;
public:
	AverageWatch() {
		time = 0;
		count = 0;
	}
	void Add(long val) {
		count++;
		time += val;
	}
	float Average() {
		return (float)time / (float)count;
	}
} global_averageWatch;


#pragma warning(disable:4244)
class Wallwatch {
	std::chrono::high_resolution_clock::time_point t_start;
public:
	void Start() {
		t_start = std::chrono::high_resolution_clock::now();
	}

	long ElapsedMilliseconds() {
		auto t_end = std::chrono::high_resolution_clock::now();
		//#pragma warning(suppress:4244)
		long millis = std::chrono::duration<float, std::milli>(t_end - t_start).count();
		return millis;
	}
};
#pragma warning(default:4244)
Wallwatch global_wall_watch;

class Program
{
public:
	static void Debug(string msg);
	float area(int radius) {
		return (float)(M_PI * ((float)radius * radius));
	}

};
class Console {
public:
	class Error {
	public:
		static void WriteLine(string msg) {
			cerr << msg << endl;
		}
	};
	static string ReadLine() {
		string line;
		getline(cin, line);

		return line;
	}
	static void WriteLine(string msg) {
		cout << msg << endl;
	}
};

class System {
public:
	class Diagnostics {
	public:
		class Trace {
		public:
			static void WriteLine(string msg) {
				cerr << msg << endl;
			}
		};
	};
};

class IReader
{
public:
	virtual string ReadLine() = 0;
	virtual void WriteLine(string output) = 0;
	virtual void AddLine(string line) = 0;
};


class ConsoleReader : public IReader
{
public:
	virtual void AddLine(string line) {}
	virtual string ReadLine()
	{
		auto line = Console::ReadLine();
		return line;
	}

	virtual void WriteLine(string output)
	{
		Console::WriteLine(output);
	}
};

class Node
{
public:
	float Win;
	int Visits;
	bool Visited;
	bool opened;
	int depth;
protected:
	bool done_sim;
	int id;
public:
	Node* parent;
	vector<Node*> children;
	static int nextId;

	Node()
	{
		this->id = Node::nextId++;
	}

	void Copy(Node* from) {
		this->depth = from->depth + 1;
	}
	void New() {
		Win = 0;
		Visits = 0;
		Visited = false;
		opened = false;
		depth = 0; 
		parent = NULL; 
		children.clear();
		done_sim = false;
	}

	virtual void Debug(int indent) = 0;

	~Node() {
		for (auto child : children) {
			delete child;
		}
	}

	virtual void open() = 0;
	virtual void do_sim() = 0;
	virtual float result(class Evaluator* evaluator) = 0;
	virtual float uct_value() = 0;
	virtual Node* pick_random() = 0;
};

int Node::nextId = 0;


class IGameInterface {
public:
	virtual Node* GetStartNode() = 0;
};

class Evaluator {
public:
	virtual float Evaluate(Node* node) = 0;
};


class MonteCarloTreeSearch
{
public:
	int rolloutsProcessed;
	bool UseTimer;
	int MaxDepth;
	int MaxRollouts;
	int MaxTime;

	bool resources_left();
	Node* monte_carlo_tree_search(Node* root, Evaluator* evaluator);
	Node* traverse(Node* node);
	Node* pick_unvisited(Node* node);
	float rollout(Node* node, Evaluator* evaluator);
	bool is_terminal(Node* node);
	Node* rollout_policy(Node* node);
	Node* pick_random(Node* node);
	void backpropagate(Node* node, float result);
	bool is_root(Node* node);
	void update_stats(Node* node, float result);
	Node* best_child(Node* node);
	Node* best_uct(Node* node);
	bool fully_expanded(Node* node);
	Node* Solve(IGameInterface* game, Evaluator* evaluator);
};


struct Vector
{
	static const float ToRad;// = (float)(M_PI / 180.0);

public:
	float X;
	float Y;
	string Present();
	Vector() { X = 0; Y = 0; }
	Vector(float x, float y) { X = x; Y = y; }

	Vector(int angle) {
		float a = (float)(angle * ToRad);
		this->X = (float)(SineHP::Cos(a));
		this->Y = (float)(SineHP::Sin(a));
	}
	float Abs() { return sqrt(this->X * this->X + this->Y * this->Y); }
	Vector Plus(Vector* b) { return Vector(this->X + b->X, this->Y + b->Y); }
	Vector Minus(Vector* b) { return Vector(this->X - b->X, this->Y - b->Y); }
	Vector Mult(float c) { return  Vector(this->X * c, this->Y * c); }
	float Dot(Vector* b) { return this->X * b->X + this->Y * b->Y; }
	Vector Unit() { auto length = this->Abs(); if (length == 0) return Vector(1, 0); return this->Mult(1 / length); }
	string DebugStr() { return string("(") + to_string((int)this->X) + "," + to_string((int)this->Y) + ")"; }
};
const float  Vector::ToRad = (float)(M_PI / 180.0);



class Random {
	static const int rand_table_size = 10000;
	int index;
public:
	Random() {
		srand(1000);
		index = 0;
		for (index = 0; index < rand_table_size; index++) {
			rand_table[index] = rand();
		}
		index = 0;
	}

	/*int Next(size_t max) {
		int result = rand() % max;
		return result;
	}*/

	int Next(size_t max) {
		index++;
		if (index >= rand_table_size)
			index = 0;

		int result = rand_table[index] % max;
		return result;
	}

	size_t rand_table[rand_table_size];
};


class BotMove
{
public:
	bool shield;
	int Thrust;
	int Angle;
};
class CodersStrikeBackMove
{
public:
	BotMove* BotMoveTable[4];
};

class Pod
{
public:
	Vector pos; Vector v; Vector cp; Vector oldPos; //TODO cp skal v�re referanse
	int oldAngle, angle, id, nextCheckPointId, round;
	int shieldTimer;
	bool sim_done;
	float blockValue; //how much we changed another pods speed. Helps with blocker heuristics
	bool is_collission; //for this only, don't copy

	void New() { 
		pos.X = 0; pos.Y = 0; v.X = 0; v.Y = 0; cp.X = 0; cp.Y = 0; oldPos.X = 0; oldPos.Y = 0;
		oldAngle = -1000; angle = -1000; id = 0;
		nextCheckPointId = 1; //A bit dependent on this, as we are checking change for detecting a new round
		round = 0;
		shieldTimer = 0; sim_done = false; blockValue = 0; is_collission = false;
	}

	void Act(int thrust, int angle, bool shield, IReader* reader, bool writeOutput);
	float DistanceToGoal(bool debug = false);
	void Copy(Pod* fromPod);
	Pod() {
		New();
	}
};//Pod

class CodersStrikeBackState : public Node
{
public:
	Pod* pods[4];
	CodersStrikeBackMove* move;
	bool is_blocker_game;

	virtual float uct_value();
	virtual Node* pick_random();
	virtual void Act(IReader* reader, int botIndex);
	void Copy(CodersStrikeBackState* from);
	void New();
	CodersStrikeBackState();
	~CodersStrikeBackState() { delete pods[0]; delete pods[1]; delete pods[2]; delete pods[3]; }
	virtual void open();
	virtual void do_sim();
	void do_sim(int podId, int thrust, int angle, bool shield);
	virtual float result(Evaluator* evaluator);
	void Debug(int indent);
};

static int pod_count;

class heap {
	const static int size = 300000;
	static CodersStrikeBackState* items[size];
	static int stackptr;
	static bool is_initialized;
public:
	static bool needs_initialization() { return !is_initialized; }
	static void init() {
		is_initialized = true;
		//count down so we get the cached items at lower indices
		for (stackptr = size - 1; stackptr >= 0; stackptr--) {
			auto item = new CodersStrikeBackState();
			items[stackptr] = item;
		}
		stackptr = 0;
	}

	static CodersStrikeBackState* New() {
		assert(stackptr < size);
		auto item = items[stackptr++];
		item->New();
		return item;
	}

	static void reset() {
		stackptr = 0;
	}
};

CodersStrikeBackState* heap::items[size];
int heap::stackptr;
bool heap::is_initialized;


class CodersStrikeBack : public IGameInterface
{
	CodersStrikeBackState* state;
	static vector<float> CalcCheckPointDistances();
public:
	static int pod_count;
	static vector<float> CheckPointDistances;
	static float roundDistance;
	static Random* rnd;
	static vector<CodersStrikeBackMove*> Moves; 
	static const float mcts_constant;// = 1.41;
	static vector<Vector> checkPoints;

	float DistanceToGoal() { return this->state->pods[0]->DistanceToGoal(); }
	void Switch0And1();
	void InitState(IReader* reader);
	void CreateMoves();
	static void AddCheckPoint(int i, Vector* checkPoint);
	void UpdateState(IReader* reader);
	Node* GetStartNode() { return this->state; }
	void CopyUpdatedState(CodersStrikeBack* game0);
	~CodersStrikeBack() {}

	CodersStrikeBack(int pod_count) {
		rnd = new Random();

		CodersStrikeBack::pod_count = pod_count;
		if (heap::needs_initialization()) {
			heap::init();
		}
		this->state = new CodersStrikeBackState();
	}
};

int CodersStrikeBack::pod_count;
const float CodersStrikeBack::mcts_constant = (float)1.41;
Random* CodersStrikeBack::rnd = NULL;
vector<Vector> CodersStrikeBack::checkPoints;
vector<float> CodersStrikeBack::CheckPointDistances;
float CodersStrikeBack::roundDistance;
vector<CodersStrikeBackMove*> CodersStrikeBack::Moves;

void CodersStrikeBackState::New() {
	Node::New();
	Visits = 0;
	Win = 0;
	Visited = false;
	opened = false;
	is_blocker_game = false;

	//Todo fjerne, kopierer over senere
	for (int podIter = 0; podIter < CodersStrikeBack::pod_count; podIter++)
	{
		this->pods[podIter]->New();
	}
}

CodersStrikeBackState::CodersStrikeBackState() :Node() {
	for (int podIter = 0; podIter < CodersStrikeBack::pod_count; podIter++)
	{
		auto pod = new Pod();
		this->pods[podIter] = pod;
	}
}


class RacingEvaluator : public Evaluator {
public:

	virtual float Evaluate(Node* node) {
		CodersStrikeBackState* csbs = (CodersStrikeBackState*)node;
		auto pod0 = csbs->pods[0];
		auto distanceToGoal0 = pod0->DistanceToGoal();

		float AllTheWay = CodersStrikeBack::roundDistance * 3;
		auto result = (AllTheWay - distanceToGoal0) / CodersStrikeBack::roundDistance;
		return result;
	}
};

float timeUntilPossibleCollision(Pod* otherPod, Pod* pod, float r) {
	Vector oldV = pod->v;
	Vector oldPos = pod->pos;
	Vector oldOtherV = otherPod->v;
	auto distance = otherPod->pos.Minus(&pod->pos).Abs();
	Vector refenceV(pod->v.Minus(&otherPod->v));

	Vector& P0 = oldPos;
	Vector& P1 = otherPod->pos;
	Vector& V0 = oldV;
	Vector& V1 = otherPod->v;
	Vector Pr = P0.Minus(&P1);
	Vector Vr = V0.Minus(&V1);

	auto closest = 100000;
	float a = Vr.X * Vr.Y;
	float b = 2 * (Pr.X * Vr.X + Pr.Y * Vr.Y);
	float c = (float)(Pr.X * Pr.X + Pr.Y * Pr.Y - 4 * r * r);

	float disc = b * b - 4 * a * c;

	if (disc <= 0)
		return -1;

	if (disc > 0) {
		float t0 = (-b + Sqrt::sqrt1(disc)) / (2 * a);
		float t1 = (-b - Sqrt::sqrt1(disc)) / (2 * a);
		float mint = 1000;
		if (t0 > 0.0)
			mint = t0;
		if (t1 > 0.0 && t1 < mint)
			mint = t1;

		return mint;
	}
}

#undef min
bool EvaluateTarget_has_written_sim_output_this_turn = false;

class OpponentStopper : public Evaluator {
public:
	float EvaluateTarget(Pod* oppPod, Pod* myPod, Pod* mySecondPod, bool debug) {
		if (EvaluateTarget_has_written_sim_output_this_turn) {
			debug = true;
			EvaluateTarget_has_written_sim_output_this_turn = false;
		}

		float distance_between_own_pods = myPod->pos.Minus(&mySecondPod->pos).Abs();
		if (distance_between_own_pods < 1000)
			return -2.0;

		if (distance_between_own_pods < 2000)
			return -1;

		float collission_part = 0; float max_collission_part = 2.0;
		float dist_opp_part = 0; float max_dist_opp_part = 1.0;
		float dist_cp_part = 0; float max_dist_cp_part = 0.1;
		float good_angle_part = 0; float max_good_angle_part = 0.1;  //[-0.1,0,1]

		//Enemy charging
		auto oppDirection = oppPod->v.Unit();
		auto oppDirectionToMe = myPod->pos.Minus(&oppPod->pos).Unit();
		auto goodOppDirection = oppDirection.Dot(&oppDirectionToMe);
		bool the_enemy_is_coming = goodOppDirection > 0.2;
		if (debug) {
			cerr << "goodOppDirection " << goodOppDirection << endl;
			cerr << "the_enemy_is_coming " << the_enemy_is_coming << endl;
		}
		if (the_enemy_is_coming) {
			auto dist_opp = myPod->pos.Minus(&oppPod->pos).Abs();
			dist_opp_part = min((float)(1 / dist_opp) * 800, max_dist_opp_part);  //800 er minimum distanse til opponenten
			dist_opp_part += max_dist_cp_part; //so we don't get a worse value for pursuing the enemy
			dist_opp_part += max_good_angle_part;//so we don't get a worse value for pursuing the enemy
			if (debug) {
				cerr << "dist_opp " << dist_opp << endl;
				cerr << "dist_opp_part0 " << dist_opp_part << endl;
				cerr << "dist_opp_part1 " << dist_opp_part << endl;
			}
		}
		else {	//Go to checkpoing, adjust angle
			int next2CheckPointIndex = (static_cast<unsigned long long>(oppPod->nextCheckPointId) + 1) % CodersStrikeBack::checkPoints.size();
			if (oppPod->round == 2) {
				next2CheckPointIndex = 0;
			}

			auto cp = CodersStrikeBack::checkPoints[next2CheckPointIndex];
			auto cpDist = myPod->pos.Minus(&cp).Abs();
			dist_cp_part = min((float)(1 / cpDist) * 500 * 0.1f, max_dist_cp_part);   //max er 1/2000 *200 = 0.1
			if (debug) {
				cerr << "next2CheckPointIndex " << next2CheckPointIndex << endl;
				cerr << "cpDist " << cpDist << endl;
				cerr << "dist cp part " << dist_cp_part << endl;
			}

			if (cpDist < 500) {
				int next1CheckPointIndex = (static_cast<unsigned long long>(oppPod->nextCheckPointId) + 1) % CodersStrikeBack::checkPoints.size();
				auto cp1 = CodersStrikeBack::checkPoints[next2CheckPointIndex];
				if (debug) {
					cerr << "cp ok, adjust angle, until he is coming" << endl;
				}

				float good_angle_part = 0;
				auto myAngleVector = Vector(myPod->angle);
				auto myDirectionToOpp = oppPod->pos.Minus(&myPod->pos).Unit();
				auto myGoodAngle = myAngleVector.Dot(&myDirectionToOpp);
				good_angle_part = myGoodAngle * 0.1 + 0.1;
			}
		}
		if (oppPod->blockValue > 0 && debug) {
			cerr << " BlockValue " << oppPod->blockValue << endl;
		}

		collission_part = min(max_collission_part, oppPod->blockValue * 0.0001F);

		float result = dist_cp_part + dist_opp_part + collission_part + good_angle_part;
		if (debug) {
			cerr << "dist_cp_part " << dist_cp_part << endl;
			cerr << "dist_opp_part " << dist_opp_part << endl;
			cerr << "good_angle_part " << good_angle_part << endl;
			cerr << "collission_part " << collission_part << endl;
			cerr << "result " << result << endl;
		}

		return result;

	}

	virtual float Evaluate(Node* node) {
		CodersStrikeBackState* csbs = (CodersStrikeBackState*)node;
		auto myPod = csbs->pods[0];

		auto oppPod2 = csbs->pods[2];
		auto dist2 = oppPod2->DistanceToGoal();

		auto oppPod3 = csbs->pods[3];
		auto dist3 = oppPod3->DistanceToGoal();

		auto oppPod = oppPod2;
		if (dist3 < dist2)
			oppPod = oppPod3;

		auto mySecondPod = csbs->pods[1];

		auto result = EvaluateTarget(oppPod, myPod, mySecondPod, false);
		return result;
	}

	OpponentStopper() {
	}
};


class GoToPositionEvaluator : public Evaluator {
public:
	Vector Target;

	virtual float Evaluate(Node* node);
};

class GoToPositionActor {
public:
	Vector Target;

	void Act(double diff, IReader* reader)
	{
		auto speed = diff / 25.0;
		if (speed < 2) {
			speed = 2.0;
			if (diff < 10)
				speed = 1;
			if (diff < 2)
				speed = 0;
		}

		cerr << "Speed: " << speed << endl;
		if (speed > 100)
			speed = 100;


		string output = to_string((int)Target.X) + " " + to_string((int)Target.Y) + " " + to_string((int)speed);
		cout << output << endl;

		reader->AddLine(output);
	}
};


void TestOpponentStopper() {
	MonteCarloTreeSearch* solver = new MonteCarloTreeSearch();
	solver->MaxDepth = 6; solver->MaxRollouts = 5000;
	int botCount = 2;

	auto game0 = new CodersStrikeBack(botCount);
	auto reader = new ConsoleReader();

	game0->InitState(reader);


	while (true)
	{
		global_timer = 0;

		global_watch.Start();
		global_wall_watch.Start();

		Stopwatch watch; watch.Start();
		game0->UpdateState(reader);

		CodersStrikeBackState* game0start = (CodersStrikeBackState*)game0->GetStartNode();
		{
			OpponentStopper evaluator;
			auto bestNext = solver->Solve(game0, &evaluator);
			auto res = bestNext->result(&evaluator);
			((CodersStrikeBackState*)bestNext)->Act(reader, 0);
		}

		int cpu = global_watch.CpuMilliseconds(true);
		cerr << "Both moves Cpu time " << cpu
			<< " Wall clock time " << global_wall_watch.ElapsedMilliseconds() << endl;



		global_round++;
	}
}

class Solver {
protected:
	string action;
public:
	string GetAction() { return action; }
};


class FirstRoundSolver : public Solver {
public:
	bool Solve(int round)
	{
		if (round == 0) {
			auto cp = CodersStrikeBack::checkPoints[1];
			action = to_string((int)cp.X) + " " + to_string((int)cp.Y) + " 100";
			return true;
		}
		return false;
	}


};

class BoosterFirstRoundSolver : public Solver {
	int boostLeft = 1;
public:
	bool Solve()
	{
		if (!boostLeft)
			return false;

		boostLeft = false;
		auto cp = CodersStrikeBack::checkPoints[1];
		action = to_string((int)cp.X) + " " + to_string((int)cp.Y) + " BOOST";
		return true;
	}
};


void AreWeLeading(CodersStrikeBackState* game0start, bool& weAreLeading, bool& pod0IsBeforepod1, bool& pod2IsBeforePod3)
{
	weAreLeading = false;
	auto pod0Left = game0start->pods[0]->DistanceToGoal();
	auto pod1Left = game0start->pods[1]->DistanceToGoal();
	auto pod2Left = game0start->pods[2]->DistanceToGoal();
	auto pod3Left = game0start->pods[3]->DistanceToGoal();
	float closest = 10000000;
	int closestIndex = -1;

	for (int index = 0; index < 4; index++) {
		float dist = game0start->pods[index]->DistanceToGoal();
		if (dist < closest) {
			closest = dist;
			closestIndex = index;
		}
	}

	if (closestIndex < 2) {
		//cerr << "We are leading" << endl;
		weAreLeading = true;
	}
	else {
		//cerr << "They are leading " << endl;
	}

	pod0IsBeforepod1 = (pod0Left < pod1Left);
	pod2IsBeforePod3 = (pod2Left < pod3Left);
}

void run(IReader* reader) {
	MonteCarloTreeSearch* solver = new MonteCarloTreeSearch();
	int botCount = 4;
	auto game0 = new CodersStrikeBack(botCount);
	auto game1 = new CodersStrikeBack(botCount);

	game0->InitState(reader);
	BoosterFirstRoundSolver boosterFirstRoundSolver0;
	BoosterFirstRoundSolver boosterFirstRoundSolver1;

	int totalRollouts = 0;

	while (true)
	{
		EvaluateTarget_has_written_sim_output_this_turn = false;
		global_timer = 0;

		global_watch.Start();
		global_wall_watch.Start();

		Stopwatch watch; watch.Start();
		game0->UpdateState(reader);
		CodersStrikeBackState* game0start = (CodersStrikeBackState*)game0->GetStartNode();

		bool weAreLeading, pod0IsBeforepod1, pod2IsBeforePod3;
		AreWeLeading(game0start, weAreLeading, pod0IsBeforepod1, pod2IsBeforePod3);

		SetPodNames("game0", game0start);
		{
			bool solved = boosterFirstRoundSolver0.Solve();
			if (solved) {
				string action = boosterFirstRoundSolver0.GetAction();
				cout << action << endl;
				auto pod = game0start->pods[0];
				string line = to_string((int)pod->pos.X) + " " + to_string((int)pod->pos.Y) + " 0 0 0 0 1";
				reader->AddLine(line);
			}
			else {
				//Pod0
				solver->MaxDepth = 5; solver->MaxRollouts = 15000;
				solver->UseTimer = true; solver->MaxTime = 30;

				RacingEvaluator evaluator;
				auto bestNext = solver->Solve(game0, &evaluator);
				((CodersStrikeBackState*)bestNext)->Act(reader, 0);
				auto distanceLeft = ((CodersStrikeBackState*)bestNext)->pods[0]->DistanceToGoal();
				cerr << "game0 round " << game0start->pods[0]->round
					<< " Rollouts: " << solver->rolloutsProcessed
					<< " Distance left" << distanceLeft << endl;
				totalRollouts += solver->rolloutsProcessed;
#ifdef PERFMON
				if (distanceLeft < 100)
					break;
#endif
			}
		}

		{
			boosterFirstRoundSolver1;
			bool solved = boosterFirstRoundSolver1.Solve();
			if (solved) {
				string action = boosterFirstRoundSolver1.GetAction();
				cout << action << endl;
				auto pod = game0start->pods[0];
				string line = to_string((int)pod->pos.X) + " " + to_string((int)pod->pos.Y) + " 0 0 0 0 1";
				reader->AddLine(line);
				reader->AddLine(line);
				reader->AddLine(line);
			}
			else {

				//Pod1
				game1->CopyUpdatedState(game0); 
				CodersStrikeBackState* game1start = (CodersStrikeBackState*)game1->GetStartNode();

				SetPodNames("game1", game0start);
				game1start->is_blocker_game = true;
				auto pod = game1start->pods[0];

				if (!solved) {
					solver->MaxDepth = 5; solver->MaxRollouts = 15000;
					solver->UseTimer = true; solver->MaxTime = 70;
					Node* bestNext = NULL;

					//cerr << "pod round " << pod->round << endl;
					int oppPodIndex = 3;
					if (pod2IsBeforePod3) {
						oppPodIndex = 2;
					}
					auto myPod = ((CodersStrikeBackState*)game1start)->pods[0];
					auto oppPod = ((CodersStrikeBackState*)game1start)->pods[oppPodIndex];
					auto racingPod = ((CodersStrikeBackState*)game1start)->pods[1];

					cerr << "myPod->round " << myPod->round << endl;
					cerr << "myPod->nextCheckPointId " << myPod->nextCheckPointId << endl;
					if (weAreLeading || ((myPod->round == 0) && (myPod->nextCheckPointId == 1))) {
						cerr << "We are leading, or early first round" << endl;
						RacingEvaluator evaluator;
						bestNext = solver->Solve(game1, &evaluator);
						auto res = bestNext->result(&evaluator);
					}
					else {
						cerr << "OpponentStopper" << endl;
						OpponentStopper evaluator;
						bestNext = solver->Solve(game1, &evaluator);
						auto res = bestNext->result(&evaluator);

						evaluator.EvaluateTarget(oppPod, myPod, racingPod, true);
					}

					//cerr << "value of opponent stopper evaluator" << res << endl;

					auto distanceLeft = ((CodersStrikeBackState*)bestNext)->pods[0]->DistanceToGoal();
					cerr << "game1 round " << game0start->pods[0]->round
						<< " Distance left" << distanceLeft
						<< " Rollouts: " << solver->rolloutsProcessed
						<< endl;
					totalRollouts += solver->rolloutsProcessed;


					((CodersStrikeBackState*)bestNext)->Act(reader, 0);
#ifdef PERFMON
					((CodersStrikeBackState*)bestNext)->Act(reader, 2);
					((CodersStrikeBackState*)bestNext)->Act(reader, 3);
#endif
				}
			}
		}
		//Act on the opponent pods also, so we can simulate

		int cpu = global_watch.CpuMilliseconds(true);
		cerr << "Both moves Cpu time " << cpu
			<< " Wall clock time " << global_wall_watch.ElapsedMilliseconds() << endl;

		global_averageWatch.Add(cpu);

		global_round++;
		cerr << "Round" << global_round << endl;

		//cerr << "Biggest sqrt x " << global_sqrt.biggest_x << endl;
	}

	cerr << "totalRollouts" << totalRollouts << endl;
}


class SimulationReader : public IReader
{
	vector<string> lines;
public:
	virtual string ReadLine()
	{
		string line = lines.front();
		lines.erase(lines.begin()); 
		return line;
	}

	virtual void AddLine(string line)
	{
		lines.push_back(line);
	}

	virtual void WriteLine(string output)
	{
		cerr << "Simulation output: " << output << endl;
	}
};


void Test4Bots()
{
	auto solver = new MonteCarloTreeSearch();
	solver->UseTimer = false;
	solver->MaxDepth = 5; solver->MaxRollouts = 1300;	

	auto game0 = new CodersStrikeBack(4);
	auto game1 = new CodersStrikeBack(4);
	auto reader = new SimulationReader();
	reader->AddLine("3");
	reader->AddLine("3");
	reader->AddLine("1000 1000");
	reader->AddLine("10000 1000");
	reader->AddLine("10000 5000");
	reader->AddLine("1000 1000 84 -2 348 1");
	reader->AddLine("10000 1000 84 -2 359 1");
	reader->AddLine("10000 1000 84 -2 359 1");
	reader->AddLine("10000 1000 84 -2 359 1");
	game0->InitState(reader);

	bool arrived = false;
	int round = 0;

	Stopwatch watch;
	watch.Start();

	while (!arrived)
	{
		RacingEvaluator Evaluator;

		game0->UpdateState(reader);
		game1->CopyUpdatedState(game0);
		{
			auto bestNext = solver->Solve(game0, &Evaluator);
			((CodersStrikeBackState*)bestNext)->Act(reader, 0);
			auto distanceLeft = game0->DistanceToGoal();
			arrived = distanceLeft < checkPointRadius;

			Program::Debug("Distance left: " + to_string(distanceLeft));
		}

		{
			auto bestNext = solver->Solve(game1, &Evaluator);
			((CodersStrikeBackState*)bestNext)->Act(reader, 0);

			//for the bots
			((CodersStrikeBackState*)bestNext)->Act(reader, 2);
			((CodersStrikeBackState*)bestNext)->Act(reader, 3);
		}

		if (round++ >= 500)
			break;
	}

	watch.Stop();
	Program::Debug(string("Rounds ") + to_string(round) + " Time " + to_string(watch.CpuMilliseconds()) + " Average: " + to_string(watch.CpuMilliseconds() / (float)round));
}



void TestSingleplayer()
{
	auto solver = new MonteCarloTreeSearch();
	solver->UseTimer = false;
	solver->MaxDepth = 6; solver->MaxRollouts = 1000;
	auto game0 = new CodersStrikeBack(1);
	auto game1 = new CodersStrikeBack(1);
	auto reader = new SimulationReader();
	reader->AddLine("3");
	reader->AddLine("3");
	reader->AddLine("1000 1000");
	reader->AddLine("10000 1000");
	reader->AddLine("10000 5000");
	reader->AddLine("1000 1000 84 -2 348 1");

	game0->InitState(reader);

	bool arrived = false;
	int round = 0;

	Stopwatch watch;
	watch.Start();

	while (!arrived)
	{
		RacingEvaluator evaluator;
		game0->UpdateState(reader);
		auto bestNext = solver->Solve(game0, &evaluator);
		((CodersStrikeBackState*)bestNext)->Act(reader, 0);
		bestNext->parent->Debug(0);
		auto distanceLeft = game0->DistanceToGoal();
		arrived = distanceLeft < checkPointRadius;

		Program::Debug("Distance left: " + to_string(distanceLeft));
	}

	watch.Stop();
	Program::Debug(string("Rounds ") + to_string(round) + " Time " + to_string(watch.CpuMilliseconds()) + " Average: " + to_string(watch.CpuMilliseconds() / (float)round));
}

void TestCollision()
{
	auto solver = new MonteCarloTreeSearch();
	solver->UseTimer = false;

	//shield i f�rste
	solver->MaxDepth = 6; solver->MaxRollouts = 5000; //Rounds 146  Time 7067 Average: 48,4041095890411

	int botCount = 2;
	auto game0 = new CodersStrikeBack(botCount);
	auto game1 = new CodersStrikeBack(botCount);
	auto reader = new SimulationReader();
	reader->AddLine("3");
	reader->AddLine("2");
	reader->AddLine("5000 5000");
	reader->AddLine("10000 5000");
	reader->AddLine("5000 5000 0 0 0 1");
	reader->AddLine("10000 5000 0 0 180 0");

	game0->InitState(reader);

	bool arrived = false;
	int round = 0;

	Stopwatch watch;
	watch.Start();

	while (!arrived)
	{

		{

			//Jeg satt denne f�rst, fordi jeg er redd for at hvis raceren er f�rst, kan denne blokkeren f� hastighet i feil retning
			//f�r den f�r sjansen til � gj�re egne valg
			game0->UpdateState(reader);

			CodersStrikeBackState* game0start = (CodersStrikeBackState*)game0->GetStartNode();
			SetPodNames("game0", game0start);
			OpponentStopper evaluator;
			//evaluator.target2 = 1;
			auto bestNext = solver->Solve(game0, &evaluator);
			auto res = bestNext->result(&evaluator);
			//cerr << "value of opponent stopper evaluator" << res << endl;

			((CodersStrikeBackState*)bestNext)->Act(reader, 0);
			auto state = (CodersStrikeBackState*)bestNext;
			//DrawCircle(state->pods[0]->pos, "Blocker");
		}

		{
			game1->CopyUpdatedState(game0);
			CodersStrikeBackState* game1start = (CodersStrikeBackState*)game1->GetStartNode();
			SetPodNames("game1", game1start);
			auto distanceLeft = game1->DistanceToGoal();
			Program::Debug(string("Distance left: ") + to_string(distanceLeft));

			RacingEvaluator evaluator;
			auto bestNext = solver->Solve(game1, &evaluator);
			((CodersStrikeBackState*)bestNext)->Act(reader, 0);
			//bestNext->parent->Debug(0);
			//game1->UpdateStateWithPodRound(bestNext);

			auto state = (CodersStrikeBackState*)bestNext;
			//DrawCircle(state->pods[0]->pos, "Racer");
		}

	}
	watch.Stop();
	Program::Debug(string("Rounds ") + to_string(round) + " Time " + to_string(watch.CpuMilliseconds())
		+ " Average: " + to_string(watch.CpuMilliseconds() / (float)round));

}


//Note: empties line
vector<string> Split(string& line, std::string delimiter = " ") {
	size_t pos = 0;
	std::string token;
	vector<string> result;
	while ((pos = line.find(delimiter)) != std::string::npos) {
		token = line.substr(0, pos);
		result.push_back(token);
		line.erase(0, pos + delimiter.length());
	}
	result.push_back(line);
	return result;
}


class TestReader : public IReader
{
	int lineIndex = 0;
	vector<string> lines;
public:

	void SetLines(string input)
	{
		this->lines = Split(input, "\r\n");

	}

	string ReadLine()
	{
		//string testInput = lines[lineIndex++]->Trim();
		string testInput = lines[lineIndex++];
		return testInput;
	}

	void WriteLine(string output)
	{
		//throw new NotImplementedException();
	}

	void AddLine(string line)
	{
		//throw new NotImplementedException();
	}
};

void BasicTest()
{
	auto solver = new MonteCarloTreeSearch();
	solver->MaxDepth = 2;
	solver->UseTimer = false;
	solver->MaxRollouts = 10000;

	auto game = new CodersStrikeBack(1);
	auto input =
		"3\r\n\
2\r\n\
1000 1000\r\n\
2000 1000\r\n\
1000 1000 0 0 0 1\r\n\
1000 1000 0 0 0 1\r\n\
1000 1000 0 0 0 1\r\n\
1000 1000 0 0 0 1";
	auto reader = new TestReader();
	reader->SetLines(input);
	game->InitState(reader);
	game->UpdateState(reader);
	RacingEvaluator evaluator;

	auto bestNext = solver->Solve(game, &evaluator);
	((CodersStrikeBackState*)bestNext)->Act(reader, 0);
}


float GoToPositionEvaluator::Evaluate(Node* node) {
	auto state = (CodersStrikeBackState*)node;
	auto myPos = state->pods[0]->pos;
	auto mySpeedV = state->pods[0]->v;


	//1 er bra. 0 er d�rlig
	auto diff = this->Target.Minus(&myPos).Abs();
	auto speed = mySpeedV.Abs();

	//max bad position and speed
	auto maxBad = Vector(16000, 9000).Abs() + Vector(1000, 1000).Abs();
	auto badVal = (diff + speed) / maxBad;
	auto goodVal = 1.0 - badVal;
	return (float)goodVal;
};

void GoToPositionTest() {
	auto solver = new MonteCarloTreeSearch();
	solver->UseTimer = false;

	//shield i f�rste
	solver->MaxDepth = 5; solver->MaxRollouts = 1300; //Rounds 146  Time 7067 Average: 48,4041095890411

	auto game0 = new CodersStrikeBack(2);
	auto game1 = new CodersStrikeBack(2);
	auto reader = new SimulationReader();
	reader->AddLine("3");
	reader->AddLine("2");
	reader->AddLine("1000 1000");
	reader->AddLine("2000 1000");
	/*reader->AddLine("1000 1000 0 0 0 1");
	reader->AddLine("2000 2000 0 0 180 0");
	reader->AddLine("10000 1000 84 -2 359 1");
	reader->AddLine("10000 1000 84 -2 359 1");*/

	reader->AddLine("10112 4754 0 0 108 1");
	reader->AddLine("4191 4828 -50 296 39 1");
	reader->AddLine("13132 4632 48 -248 212 1");
	reader->AddLine("7727 2176 -441 124 144 2");

	game0->InitState(reader);

	bool arrived = false;
	int round = 0;

	Stopwatch watch;
	watch.Start();

	while (!arrived)
	{
		game0->UpdateState(reader);
		game1->CopyUpdatedState(game0);
		{
			GoToPositionEvaluator evaluator;
			evaluator.Target.X = 10000;
			evaluator.Target.Y = 5000;

			//auto distanceLeft = game0->DistanceToGoal();

			/*Program::Debug(string("Distance left: ") + to_string(distanceLeft));
			arrived = distanceLeft < 600;
			if (arrived)
				break;*/

			CodersStrikeBackState* startNode = (CodersStrikeBackState*)game0->GetStartNode();
			Vector& pos = startNode->pods[0]->pos;
			auto diff = evaluator.Target.Minus(&pos).Abs();
			if (diff > 1000) {
				auto bestNext = solver->Solve(game0, &evaluator);

				auto value = evaluator.Evaluate(bestNext); //Maybe check nextX, nextY (after do_sim instead)
				if (value > 0.99) {
					arrived = true;
					break;
				}

				((CodersStrikeBackState*)bestNext)->Act(reader, 0);
				((CodersStrikeBackState*)bestNext)->Act(reader, 1);
				//bestNext->parent->Debug(0);

				//game0->UpdateStateWithPodRound(bestNext);
			}
			else {
				auto speed = diff / 25.0;
				cout << evaluator.Target.X << " " << evaluator.Target.Y << " " << speed << endl;
			}
		}

	}
	watch.Stop();
	Program::Debug(string("Rounds ") + to_string(round) + " Time " + to_string(watch.CpuMilliseconds())
		+ " Average: " + to_string(watch.CpuMilliseconds() / (float)round));
}

void TestDebug();

int main()
{
	IReader* reader = NULL;

#ifdef PERFMON
	reader = new SimulationReader();
	reader->AddLine("3");
	reader->AddLine("2");
	reader->AddLine("5000 5000");
	reader->AddLine("10000 5000");
	reader->AddLine("5000 5000 0 0 0 1");
	reader->AddLine("10000 5000 0 0 180 0");
	reader->AddLine("5000 10000 0 0 0 1");
	reader->AddLine("10000 10000 0 0 180 0");
#else
	reader = new ConsoleReader();
#endif

	run(reader);

	cerr << "Average cpu time per round " << global_averageWatch.Average() << endl;
}



void Program::Debug(std::string msg)
{
	msg = string(" ") + msg;
	Console::Error::WriteLine(msg);
	//System::Diagnostics::Trace::WriteLine(msg);
}

bool MonteCarloTreeSearch::resources_left()
{
	this->rolloutsProcessed++;

	if (rolloutsProcessed < 5)  //Seriously, give us time to do at least a handfull
		return true;

	if (this->UseTimer)
	{
#ifndef _DEBUG
		//auto diff = this->stopwatch->CpuMilliseconds();  //TODO CPU-tid, ikke sanntid
		auto diff = global_watch.CpuMilliseconds(true);
		bool outOfTime = diff > this->MaxTime;
		if (outOfTime) {
			cerr << "Out of time time after " << this->rolloutsProcessed << " rollouts "
				<< " Cpu time: " << global_watch.CpuMilliseconds(true) << endl;

			return false;
		}
#endif
		//We also check for a max rollouts
		return this->rolloutsProcessed < this->MaxRollouts;

	}
	else
	{
		return this->rolloutsProcessed < this->MaxRollouts;
	}
}

Node* MonteCarloTreeSearch::monte_carlo_tree_search(Node* root, Evaluator* evaluator)
{
	this->rolloutsProcessed = 0;

	while (resources_left())
	{
		auto leaf = traverse(root);
		auto simulation_result = rollout(leaf, evaluator);

		backpropagate(leaf, simulation_result);
	}
	return best_child(root);
}

Node* MonteCarloTreeSearch::traverse(Node* node)
{
	while (fully_expanded(node)) //Todo..returnere f�rste som ikke er bes�kt, s� slipper man � gjenta samme operasjon rett under
	{
		node = best_uct(node);

		if (node->depth >= this->MaxDepth) //Jeg har lagt til denne
			return node;
	}

	auto child = pick_unvisited(node); // or node
	if (child == NULL)
		return node;

	return child;
}


Node* MonteCarloTreeSearch::pick_unvisited(Node* node)
{
	if (!node->opened)
	{
		node->open();
		node->opened = true;
	}

	for (auto child : node->children)
	{
		if (!child->Visited)
		{
			child->do_sim();
			//Ikke blande sammen Visited og Visits-> Selv om Visited = Visits > 0
			child->Visited = true;
			return child;
		}
	}

	return NULL;
}

float MonteCarloTreeSearch::rollout(Node* node, Evaluator* evaluator)
{
	while (!is_terminal(node))
	{
		node = rollout_policy(node);
		node->do_sim();
	}
	auto res = node->result(evaluator);
	return res;
}

bool MonteCarloTreeSearch::is_terminal(Node* node)
{
	if (node->depth >= this->MaxDepth)
		return true;

	return false;
}

Node* MonteCarloTreeSearch::rollout_policy(Node* node)
{
	return pick_random(node);
}



Node* MonteCarloTreeSearch::pick_random(Node* node)
{
	auto picked = node->pick_random();
	//picked->do_sim(); gj�res etterp�
	return picked;
}

void MonteCarloTreeSearch::backpropagate(Node* node, float result)
{
	if (is_root(node))
	{
		update_stats(node, result);
		return;
	}
	update_stats(node, result);
	auto parent = node->parent;
	backpropagate(parent, result);
}

bool MonteCarloTreeSearch::is_root(Node* node)
{
	return node->parent == NULL;
}

void MonteCarloTreeSearch::update_stats(Node* node, float result)
{
	node->Visits += 1;
	node->Win += result;
}

Node* MonteCarloTreeSearch::best_child(Node* node)
{
	int best_so_far = -1000000;
	Node* favourite_child = NULL;
	for (auto child : node->children)
	{
		if (child->Visits > best_so_far)
		{
			best_so_far = child->Visits;
			favourite_child = child;
		}
	}
	return favourite_child;
}
Node* MonteCarloTreeSearch::best_uct(Node* node)
{
	Node* best_ucts_child_so_far = NULL;

	float best_ucts_so_far = best_ucts_so_far = -1000000;

	for (auto child : node->children)
	{
		auto ucts = child->uct_value();
		if (ucts > best_ucts_so_far)
		{
			best_ucts_child_so_far = child;
			best_ucts_so_far = ucts;
		}
	}

	return best_ucts_child_so_far;
}


bool MonteCarloTreeSearch::fully_expanded(Node* node)
{
	if (!node->opened)
		return false;

	if (node->children.size() == 0)
		return true;

	for (auto child : node->children)
	{
		if (!child->Visited)
			return false;
	}
	return true;
}

Node* MonteCarloTreeSearch::Solve(IGameInterface* game, Evaluator* evaluator)
{
	auto startNode = game->GetStartNode();
	Node* bestChild = monte_carlo_tree_search(startNode, evaluator);
	return bestChild;
}


void CodersStrikeBack::Switch0And1()
{
	auto pod0 = this->state->pods[0];
	this->state->pods[0] = this->state->pods[1];
	this->state->pods[1] = pod0;
}



float CodersStrikeBackState::uct_value()
{
	if (parent == NULL)
		return 1000000; //start node has noparent


	if (Visits == 0)
		return 1000000;

	//Usikker p� om vi skal ha ln, log10 el log2 her, eller om det spiller noen rolle
	float log_parent_visits = (float)global_log.log(parent->Visits);
	auto uct = Win / Visits + CodersStrikeBack::mcts_constant * Sqrt::sqrt1(2 * log_parent_visits / Visits);
	return uct;
}


Node* CodersStrikeBackState::pick_random()
{
	//auto newState = new CodersStrikeBackState(this);
	auto newState = heap::New();
	newState->Copy(this);


	size_t size = CodersStrikeBack::Moves.size();
	int index = CodersStrikeBack::rnd->Next(size);
	CodersStrikeBackMove* strikeBackMove = CodersStrikeBack::Moves[index];
	newState->move = strikeBackMove;
	return newState;
}


void CodersStrikeBackState::Act(IReader* reader, int botIndex)
{
	CodersStrikeBackMove* theMove = (CodersStrikeBackMove*)(move);

	auto pod = this->pods[botIndex];
	bool shield = false;
	if (pod->is_collission)
		cerr << "COLLISSION" << endl;

	auto botMove = theMove->BotMoveTable[botIndex];
	//pod->Act(botMove->Thrust, botMove->Angle, botMove->shield, reader, true);
	pod->Act(botMove->Thrust, botMove->Angle, pod->is_collission, reader, true);
}


void CodersStrikeBackState::Copy(CodersStrikeBackState* from) {
	Node::Copy(from);

	for (int podIter = 0; podIter < CodersStrikeBack::pod_count; podIter++)
	{
		auto fromPod = from->pods[podIter];
		this->pods[podIter]->Copy(fromPod);
	}
	this->parent = from;
	this->is_blocker_game = from->is_blocker_game;
}


void CodersStrikeBackState::open()
{
	for (auto move : CodersStrikeBack::Moves)
	{
		CodersStrikeBackState* child = heap::New();
		child->Copy(this);
		child->move = move;
		this->children.push_back(child);
	}
}


void CodersStrikeBackState::do_sim(int podId, int thrust, int angle, bool shield)
{
	int collissionIndex = -1;
	Pod* pod = this->pods[podId];
	pod->oldPos = pod->pos;
	pod->oldAngle = pod->angle;
	Vector oldV = pod->v;
	Vector oldPos = pod->pos;

	pod->angle += angle;
	while (pod->angle > 360)
		pod->angle -= 360;
	while (pod->angle < -360)
		pod->angle += 360;

	auto angleVector = Vector(pod->angle);
	auto thrustVector = Vector(0, 0); //todo remove

	if (shield)
		pod->shieldTimer = 3; //todo el var det 4

	if (pod->shieldTimer > 0)
	{
		pod->shieldTimer--;
	}
	else
	{
		thrustVector = angleVector.Mult((float)thrust);
	}

	pod->v = pod->v.Plus(&thrustVector);
	pod->pos = pod->pos.Plus(&pod->v);
	pod->v = pod->v.Mult(0.85F); //friction

	Pod* collissionPod = NULL;
	bool collision = false;

	for (int podIter = 1; podIter < CodersStrikeBack::pod_count; podIter++)
	{
		auto otherPod = this->pods[podIter];
		Vector oldOtherV = otherPod->v;

		auto distance = otherPod->pos.Minus(&pod->pos).Abs();
		double r = 400;

		Vector refenceV(pod->v.Minus(&otherPod->v));
		auto max_possible_collision_radius = refenceV.Abs() + 2 * r;
		if (distance < max_possible_collision_radius)
		{
			//could be a collision
			//The following is based on a model where we put the other pod (P1,V1) in origo and made it stationary.
			//   Our pod is reduced to a point, with the speed-vectors added. The radious for the other pod is doubled
			//A ray is drawn through the other pod (the one with double radius) the ray may intersect in 2, 1 or zero points.
			Vector& P0 = oldPos;
			Vector& P1 = otherPod->pos;
			Vector& V0 = oldV;
			Vector& V1 = otherPod->v;
			Vector Pr = P0.Minus(&P1);
			Vector Vr = V0.Minus(&V1);

			auto closest = 100000;
			float a = Vr.X * Vr.Y;
			float b = 2 * (Pr.X * Vr.X + Pr.Y * Vr.Y);
			float c = (float)(Pr.X * Pr.X + Pr.Y * Pr.Y - 4 * r * r);

			float disc = b * b - 4 * a * c;
			if (disc > 0) {
				float t0 = (-b + Sqrt::sqrt1(disc)) / (2 * a);
				float t1 = (-b - Sqrt::sqrt1(disc)) / (2 * a);

				float mint = 1000000;
				if (t0 > 0.0 && t0 < 1.0)
					mint = t0;
				if (t1 > 0.0 && t1 < 1.0 && t1 < mint)
					mint = t1;

				if (mint > 0.0 && mint < 1.0) {
					collision = true;
					//Game0: pod0 is the racing pod, pod1 is the blocker. Will only act on the racing pod
					//Game1: pod0 is the blocker, pod1 is the racing pod. Will only act on the blocker
					//If the other pod is pod1/blocker, and I am pod0, don't put on the shield on the blocker
					//We don't do anything for collission when opponent is pod0 or pod1

					//also, only put on a shield if the collission moves us in the wrong direction

					//Litt hard-koding her
					if (podIter > 1) {
						if (this->is_blocker_game) {
							pod->is_collission = true;
							otherPod->is_collission = true;
						}
						else {
							auto dot = pod->v.Dot(&otherPod->v);
							if (dot < 0) {
								pod->is_collission = true;
								otherPod->is_collission = true;
							}
						}
					}

					collissionIndex = podIter;


					//Da har vi funnet den korteste tiden som gir kollisjon.
					//Kollisjonssentra
					Vector C0 = V0.Mult(mint); C0 = C0.Plus(&P0); //C0.Debug("C0");
					Vector C1 = V1.Mult(mint); C1 = C1.Plus(&P1); //C1.Debug("C1");

					Vector P0C_P1C = C1.Minus(&C0); //P0C_P1C.Debug("P0C_P1C");
					Vector P1C_P0C = Vector(-P0C_P1C.X, -P0C_P1C.Y); //P1C_P0C.Debug("P1C_P0C");
					float speed0 = V0.Abs();
					float speed1 = V1.Abs();

					Vector newV1 = P0C_P1C.Unit(); //newV1.Debug("newV1a");
					Vector newV0 = P1C_P0C.Unit(); //newV0.Debug("newV0a");

					float myMass = 1;
					float otherMass = 1;
					auto tLeft = 1 - mint;

					newV0 = newV0.Mult(speed1 * otherMass);
					newV1 = newV1.Mult(speed0 * myMass);

					Vector moveNow0 = newV0.Mult(tLeft); //moveNow0.Debug("moveNow0");
					Vector moveNow1 = newV1.Mult(tLeft); //moveNow1.Debug("moveNow1");

					auto newP0 = C0.Plus(&moveNow0);
					auto newP1 = C1.Plus(&moveNow1);


					otherPod->v = otherPod->v.Mult(0.85F); //friction
					pod->v = pod->v.Mult(0.85F);

					//pos/v after collision
					pod->v = newV0; //pod->v.Debug("V0B");
					pod->pos = newP0; //pod->pos.Debug("P0B");

					//other pos/v after collision
					otherPod->v = newV1;//otherPod->v.Debug("V1B");
					otherPod->pos = newP1; //otherPod->pos.Debug("P1B");
					otherPod->sim_done = true;

					auto dotOther = oldOtherV.Dot(&otherPod->v);  //Dette gir et stort tall hvis det er i samme retning
					otherPod->blockValue += -dotOther;

					auto dotMe = oldV.Dot(&pod->v);
					pod->blockValue += -dotMe;
				}
			}
		}
	}

	if (shield && !collision) {
		//We don't want this one. Should have a way to say "this is a very bad mover"
		//this->pods[0]->pos = Vector(1000000, 100000);
	}


	auto cp = CodersStrikeBack::checkPoints[pod->nextCheckPointId];

	if (cp.Minus(&pod->pos).Abs() < checkPointRadius)
	{
		pod->nextCheckPointId++;

		size_t checkPointNo = CodersStrikeBack::checkPoints.size();
		if (pod->nextCheckPointId >= checkPointNo)
		{
			pod->nextCheckPointId = 0;
			pod->cp = CodersStrikeBack::checkPoints[pod->nextCheckPointId];
		}
		if (pod->nextCheckPointId == 1)
		{
			pod->round++;
		}
	}
}


void CodersStrikeBackState::do_sim()
{
	if (!this->done_sim)
	{
		this->done_sim = true;
		//auto mv = this->move as CodersStrikeBackMove;
		CodersStrikeBackMove* mv = (CodersStrikeBackMove*)this->move;

		for (int botIndex = 0; botIndex < CodersStrikeBack::pod_count; botIndex++)
		{
			auto botMove = mv->BotMoveTable[botIndex];
			do_sim(botIndex, botMove->Thrust, botMove->Angle, botMove->shield);
		}
	}
}

//Baseklassens result kalles fra MonteCarloTreeSearch som kjenner til node 
float CodersStrikeBackState::result(Evaluator* evaluator)
{
	auto value = evaluator->Evaluate(this);
	return value;
}


//index 0 -> avstanden mellom 0 og 1->
vector<float> CodersStrikeBack::CalcCheckPointDistances()
{
	vector<float> checkPointDistances;

	for (int cpi = 0; cpi < CodersStrikeBack::checkPoints.size(); cpi++)
	{
		Vector& cp0 = CodersStrikeBack::checkPoints[cpi];
		Vector* cp1 = NULL;
		if (cpi < CodersStrikeBack::checkPoints.size() - 1)
		{
			cp1 = &CodersStrikeBack::checkPoints[cpi + 1];
		}
		else
		{
			cp1 = &CodersStrikeBack::checkPoints[0];
		}
		auto dist = cp0.Minus(cp1).Abs(); //Minnelekkasje. Dette m� gj�res p� stacken, eller dele opp i flere operasjoner og delete
		checkPointDistances.push_back(dist);
	}

	return checkPointDistances;
}


void Pod::Act(int thrust, int angle, bool shield, IReader* reader, bool writeOutput)
{

	auto newAngle = oldAngle + angle;
	auto angleVector = Vector(newAngle);
	angleVector = angleVector.Mult(5000);
	auto goalVector = oldPos.Plus(&angleVector);

	string output;
	if (shield) {
		output = to_string((int)goalVector.X) + " " + to_string((int)goalVector.Y) + " SHIELD";
	}
	else {
		output = to_string((int)goalVector.X) + " " + to_string((int)goalVector.Y) + " " + to_string(thrust);
	}

	if (writeOutput) {//Not for the opponent bots, which still need to be added to the simulation
		reader->WriteLine(output);
	}

	//For testing, write the exact output, for us to read back on the next iteration
	string line = to_string((int)this->pos.X) + " " + to_string((int)this->pos.Y) + " "
		+ to_string((int)this->v.X) + " " + to_string((int)this->v.Y) + " "
		+ to_string((int)newAngle) + " " + to_string(this->nextCheckPointId);
	reader->AddLine(line);

	if (id == 0)
	{
		float diffY = pos.Y - oldPos.Y;
		float diffX = pos.X - oldPos.X;
		float newAngle2 = (float)(atan2(diffY, diffX) * 180 / M_PI);
	}

}

float  Pod::DistanceToGoal(bool debug)
{
	auto roundsDistanceLeft = (2 - round) * CodersStrikeBack::roundDistance;
	Vector& cp = CodersStrikeBack::checkPoints[this->nextCheckPointId];
	auto distanceToCheckPoint = cp.Minus(&this->pos).Abs();

	auto distanceInTheRoundLeft = 0.0;
	for (int cpIndex = this->nextCheckPointId; cpIndex < CodersStrikeBack::checkPoints.size(); cpIndex++)
	{
		if (cpIndex == 0)
			break; //hvis man er p� siste strekning, s� trenger man ikke noe bidrag herfra
		distanceInTheRoundLeft += CodersStrikeBack::CheckPointDistances[cpIndex];
	}

	auto total = roundsDistanceLeft + distanceToCheckPoint + distanceInTheRoundLeft;
	return (float)total;
}

void Pod::Copy(Pod* fromPod)
{
	this->angle = fromPod->angle;
	this->cp = fromPod->cp;
	this->id = fromPod->id;
	this->nextCheckPointId = fromPod->nextCheckPointId;
	this->pos = fromPod->pos;
	this->round = fromPod->round;
	this->v = fromPod->v;
	this->shieldTimer = fromPod->shieldTimer;

	this->blockValue = fromPod->blockValue;
}



void CodersStrikeBack::InitState(IReader* reader)
{
	string lapsLine = reader->ReadLine();
	Console::Error::WriteLine(lapsLine);
	int laps = atoi(lapsLine.c_str());

	string checkPointCountLine = reader->ReadLine();
	Console::Error::WriteLine(checkPointCountLine);
	int checkpointCount = atoi(checkPointCountLine.c_str());

	for (int i = 0; i < checkpointCount; i++)
	{
		auto line = reader->ReadLine();
		Console::Error::WriteLine(line);
		vector<string> inputs = Split(line, " ");
		int checkpointX = atoi(inputs[0].c_str());
		int checkpointY = atoi(inputs[1].c_str());

		auto checkPoint = new Vector((float)checkpointX, (float)checkpointY);
		AddCheckPoint(i, checkPoint);
	}
	CodersStrikeBack::CheckPointDistances = CodersStrikeBack::CalcCheckPointDistances();

	roundDistance = 0;
	std::for_each(CodersStrikeBack::CheckPointDistances.begin(), CodersStrikeBack::CheckPointDistances.end(), [&](float n) {roundDistance += n; });
	roundDistance = (float)(accumulate(CodersStrikeBack::CheckPointDistances.begin(), CodersStrikeBack::CheckPointDistances.end(), 0.0F));

	CreateMoves();
}

//Vi lager ikke mange trekk for motstanderen, antar at de kj�rer rett fram
void CodersStrikeBack::CreateMoves()
{
	bool shield = false;
	for (int thrust0 = 0; thrust0 <= 100; thrust0 += 25) {

		for (int angle0 = -20; angle0 <= 20; angle0 += 5) {
			auto move = new CodersStrikeBackMove();
			{
				auto botMove = new BotMove();
				botMove->Angle = angle0;
				botMove->Thrust = thrust0;
				botMove->shield = shield;
				move->BotMoveTable[0] = botMove;
			}
			{
				auto botMove = new BotMove();
				botMove->Angle = 0;
				botMove->Thrust = 100;
				botMove->shield = false;

				move->BotMoveTable[1] = botMove;
				move->BotMoveTable[2] = botMove;
				move->BotMoveTable[3] = botMove;
			}
			CodersStrikeBack::Moves.push_back(move);
		}
	}
}

void CodersStrikeBack::AddCheckPoint(int i, Vector* checkPoint)
{
	CodersStrikeBack::checkPoints.push_back(*checkPoint);
}

void CodersStrikeBack::UpdateState(IReader* reader)
{
	heap::reset();
	Node* node = this->state;
	node->New();


	Node::nextId = 0;
	vector<string> inputs;
	int podId = 0;
	if (this->state == NULL) {
		assert(false);
		this->state = heap::New();
	}

	for (int i = 0; i < pod_count; i++)
	{
		auto pod = this->state->pods[i];

		string line = reader->ReadLine();
		if (line == "") {
			cerr << "Empty input, throwing exception " << endl;
			exit(1);
		}

		Console::Error::WriteLine(line);
		inputs = Split(line, " ");

		int x = atoi(inputs[0].c_str()); // x position of your pod
		int y = atoi(inputs[1].c_str()); // y position of your pod
		int vx = atoi(inputs[2].c_str()); // x speed of your pod
		int vy = atoi(inputs[3].c_str()); // y speed of your pod
		int angle = atoi(inputs[4].c_str()); // angle of your pod
		int nextCheckPointId = atoi(inputs[5].c_str()); // next check point id of your pod

		pod->pos = Vector((float)x, (float)y);
		pod->v = Vector((float)vx, (float)vy);
		pod->cp = checkPoints[nextCheckPointId];
		if (nextCheckPointId != pod->nextCheckPointId && nextCheckPointId == 1)
			pod->round++;

		pod->nextCheckPointId = nextCheckPointId;

		pod->angle = angle;
	}
}

void CodersStrikeBack::CopyUpdatedState(CodersStrikeBack* game0)
{
	Node::nextId = 0;
	vector<string> inputs;
	int podId = 0;

	Stopwatch watch; watch.Start();
	this->state = heap::New();

	for (int i = 0; i < CodersStrikeBack::pod_count; i++)
	{
		this->state->pods[i]->Copy(game0->state->pods[i]);
	}

	auto pod0 = this->state->pods[0];
	auto pod1 = this->state->pods[1];

	this->state->pods[0] = pod1;
	this->state->pods[1] = pod0;
}


void CodersStrikeBackState::Debug(int indent) {
	std::streamsize ss = std::cout.precision();
	cerr << this->id << " " << this->depth << " W " << setw(9) << this->Win << " V " << this->Visits
		<< " pos (" << this->pods[0]->pos.X << "," << this->pods[0]->pos.X << ") timer" << this->pods[0]->shieldTimer;
	if (this->move) {
		cerr << " Move (AST) ("
			<< this->move->BotMoveTable[0]->Angle << " " << this->move->BotMoveTable[0]->shield << " " << this->move->BotMoveTable[0]->Thrust
			<< ")" << endl;
	}
	else {
		cerr << " ROOT " << endl;
	}

	for (auto child : this->children) {
		for (int i = 0; i < indent; i++)
			cerr << "  ";
		child->Debug(indent + 1);
	}
}
