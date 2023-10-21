#include <bits/stdc++.h>

#define pb push_back
#define pii pair<int, int>

using namespace std;

typedef long long ll;

struct point {
    ll x, y;
};

point operator-(point p1, point p2){
    return point{p1.x - p2.x, p1.y - p2.y};
}

point operator+(point p1, point p2){
    return point{p1.x + p2.x, p1.y + p2.y};
}

ll vec_mul(point p1, point p2){
    return p1.x * p2.y - p2.x * p1.y;
}

bool overflow(ll l1, ll r1, ll l2, ll r2){
    if(l1 > r1)
        swap(l1, r1);
    if(l2 > r2)
        swap(l2, r2);
    return l1 <= r2 and r1 >= l2;
}

bool onSegment(point x, point a, point b){
    return vec_mul(x - a, b - a) == 0 and overflow(a.x, b.x, x.x, x.x) and overflow(a.y, b.y, x.y, x.y);
}

bool diffSign(ll a, ll b){
    return (a >= 0 and b <= 0) or (a <= 0 and b >= 0);
}

bool intersect(point a1, point b1, point a2, point b2){
    if(onSegment(a1, a2, b2) or onSegment(b1, a2, b2) or onSegment(a2, a1, b1) or onSegment(b2, a1, b1)){
        return 0;
    }

    ll d = vec_mul(b1 - a1, b2 - a2);
    if(d){
        if(
            diffSign( vec_mul(b1 - a2, b2 - a2), vec_mul(a1 - a2, b2 - a2) ) and 
            diffSign( vec_mul(b2 - a1, b1 - a1), vec_mul(a2 - a1, b1 - a1) )
        ) {
            return 1;
        } else {
            return 0;
        }
    } else {
        if(
            vec_mul(a2 - a1, b1 - a1) == 0 and 
            overflow(a1.x, b1.x, a2.x, b2.x) and
            overflow(a1.y, b1.y, a2.y, b2.y)
        ) {
            return 1;
        } else {
            return 0;
        }
    }
}

// -------------------------------------


struct pos {
	int x;
	int y;
	int dir;
};

bool operator<(const pos& a, const pos& b) {
	return (a.x < b.x) || (a.x == b.x && a.y < b.y) || (a.x == b.x && a.y == b.y && a.dir < b.dir);
}

bool operator==(const pos& a, const pos& b) {
	return (a.x == b.x) && (a.y == b.y) && (a.dir == b.dir);
}

bool operator!=(const pos& a, const pos& b) {
	return !(a == b);
}

ostream& operator<<(ostream& os, const pos& p) {
	return os << "{ " << p.x << " " << p.y << " " << p.dir << " }";
}

const vector<int> dx = {1, 0, -1, 0};
const vector<int> dy = {0, -1, 0, 1};

const vector<int> w1x = {1, -1, -1, -1};
const vector<int> w1y = {-1, -1, 1, -1};

const vector<int> w2x = {1, -1, 1, 1};
const vector<int> w2y = {1, 1, 1, -1};

pos go_forward(pos cur) {
	cur.x += dx[cur.dir];
	cur.y += dy[cur.dir];
	return cur;
}

pos go_right(pos cur) {
	cur.dir = (cur.dir + 1) % 4;
	return cur;
}

pos go_left(pos cur) {
	cur.dir = (cur.dir - 1 + 4) % 4;
	return cur;
}

int a, b, c, k;
pos robot;

map<pii, int> m;

set<pii> unknown;  // ?
set<pii> uw;  // ?

int ans_cost = 0;

void put(int x, int y, int t) {
	if (m[{x, y}] == 3) {
		unknown.erase({x, y});
	}

	m[{x, y}] = t;

	if (t == 1) {  // _
		// add new ?
		for (int i = 0; i < 4; ++i) {
			int nx = x + dx[i];
			int ny = y + dy[i];
			if (m[{nx, ny}] == 0) {
				m[{nx, ny}] = 3;
				unknown.insert({nx, ny});
			}
		}
	} else if (t == 2) {  // #
		// nothing
	} else if (t == 3) {  // ?
		unknown.insert({x, y});
	}
}

bool ok(pos cur) {
	return (m[{cur.x, cur.y}] == 1) || (m[{cur.x, cur.y}] == 3);
}


map<pos, int> d;
map<pii, int> d_xy;
map<pos, pair<pos, int>> pred;

void dikstra() {
	d.clear();
	pred.clear();
	d[robot] = 0;
	set<pair<int, pos>> q;
	q.insert({0, robot});
	while (!q.empty()) {
		pos cur = q.begin()->second;
		q.erase(q.begin());

		if (m[{cur.x, cur.y}] != 1) {
			continue;
		}

		pos nxt;

		// forward
		nxt = go_forward(cur);
		if (ok(nxt) && ((d.find(nxt) == d.end()) || (d[nxt] > d[cur] + a))) {
			q.erase({d[nxt], nxt});
			d[nxt] = d[cur] + a;
			pred[nxt] = make_pair(cur, 0);
			q.insert({d[nxt], nxt});
		}

		// turn right
		nxt = go_right(cur);
		if (ok(nxt) && ((d.find(nxt) == d.end()) || (d[nxt] > d[cur] + b))) {
			q.erase({d[nxt], nxt});
			d[nxt] = d[cur] + b;
			pred[nxt] = make_pair(cur, 1);
			q.insert({d[nxt], nxt});
		}

		// turn left
		nxt = go_left(cur);
		if (ok(nxt) && ((d.find(nxt) == d.end()) || (d[nxt] > d[cur] + b))) {
			q.erase({d[nxt], nxt});
			d[nxt] = d[cur] + b;
			pred[nxt] = make_pair(cur, 2);
			q.insert({d[nxt], nxt});
		}
	}

	d_xy.clear();
	for (auto e : d) {
		if ((d_xy.find({e.first.x, e.first.y}) == d_xy.end()) || (d_xy[{e.first.x, e.first.y}] > e.second)) {
			d_xy[{e.first.x, e.first.y}] = e.second;
		}
	}
}


bool check(int x1, int y1, int x2, int y2, int cx, int cy) {
	point p1 = point({x1 * 2 + 1, y1 * 2 + 1});
	point p2 = point({x2 * 2 + 1, y2 * 2 + 1});
	point pc = point({cx * 2 + 1, cy * 2 + 1});

	for (int i = 0; i < 4; ++i) {
		point p3 = point({pc.x + w1x[i], pc.y + w1y[i]});
		point p4 = point({pc.x + w2x[i], pc.y + w2y[i]});
		if (intersect(p1, p2, p3, p4)) {
			return true;
		}
	}
	return false;
}

void ask(int x, int additional) {
	cout << x;
	if (x == 2) {
		cout << ", " << additional;
	}
	cout << endl;

	if (x == 1) {
		ans_cost += a;
		int res;
		cin >> res;
		if (res) {
			// go next
			robot = go_forward(robot);
		}
	} else if (x == 2) {
		ans_cost += b;
		int res;
		cin >> res;
		if (additional == 0) {
			robot = go_left(robot);
		} else {
			robot = go_right(robot);
		}
	} else {
		ans_cost += c;
		// no cin
	}
}


signed main() {
	{
		vector<int> v;
		v.resize(8);
		for (int i = 0; i < 8; ++i) {
			string s;
			cin >> s;
			if (i != 7) s.pop_back();
			v[i] = std::stoi(s);
		}
		pii p1, p2;
		p1.first = v[0];
		p1.second = v[1];
		p2.first = v[2];
		p2.second = v[3];
		a = v[4];
		b = v[5];
		c = v[6];
		k = v[7];

		robot.x = p1.first;
		robot.y = p1.second;
		p2.first -= p1.first;
		p2.second -= p1.second;
		for (int i = 0; i < 4; ++i) {
			if ((dx[i] == p2.first) && (dy[i] == p2.second)) {
				robot.dir = i;
				break;
			}
		}
	}

	// put(1, 0, 2);
	// put(0, 1, 2);

	put(robot.x, robot.y, 1);


	int pre_fire = 0;
	while (!unknown.empty()) {
		// dist to all
		dikstra();

		int cost_fire = c;
		int cost_go = 0;

		// calc cost
		uw.clear();
		unknown.clear();
		for (auto e : m) {
			if (e.second == 2 || e.second == 3) {
				uw.insert({e.first.first, e.first.second});
			}
			if (e.second == 3) {
				unknown.insert({e.first.first, e.first.second});
			}
		}

		for (auto e : unknown) {
			if ((abs(e.first - robot.x) > k) || (abs(e.second - robot.y) > k)) continue;
			bool see = true;
			for (auto wallcell : uw) {
				if (e == wallcell) continue;
				if (check(robot.x, robot.y, e.first, e.second, wallcell.first, wallcell.second)) {
					see = false;
					break;
				}
			}
			if (see) {
				cost_go += 2 * d[{e.first, e.second}] + 4 * b;
			}
		}


		if (cost_go > cost_fire && !pre_fire) {
			// fire
			pre_fire = 1;
			ask(3, 0);

			// cin...
			vector<string> mas;
			mas.resize(2*k + 1);
			for (int i = 0; i < 2*k + 1; ++i) {
				cin >> mas[i];
			}
			reverse(mas.begin(), mas.end());

			for (int i = 0; i < 2*k + 1; ++i) {
				for (int j = 0; j < 2*k + 1; ++j) {
					int nx = robot.x + j - k;
					int ny = robot.y + i - k;

					if (mas[i][j] == '_') {
						put(nx, ny, 1);
					}
				}
			}


			vector<pii> surewall;
			for (auto e : unknown) {
				if ((abs(e.first - robot.x) > k) || (abs(e.second - robot.y) > k)) continue;
				bool see = true;
				for (auto wallcell : uw) {
					if (e == wallcell) continue;
					if (check(robot.x, robot.y, e.first, e.second, wallcell.first, wallcell.second)) {
						see = false;
						break;
					}
				}
				if (see) {
					surewall.push_back(e);
				}
			}

			for (auto e : surewall) {
				put(e.first, e.second, 2);
			}
		} else {
			pre_fire = 0;
			// go
			pos minpos;
			int mindist = -1;
			for (auto e : d) {
				if (m[{e.first.x, e.first.y}] != 3) continue;
				if (e.second == 0) continue;
				if ((robot.x == e.first.x) && (robot.y == e.first.y)) continue;
				if ((mindist == -1) || (mindist > e.second)) {
					mindist = e.second;
					minpos = e.first;
				}
			}

			pair<pos, int> pre = pred[minpos];
			while (pre.first != robot) {
				minpos = pre.first;
				pre = pred[minpos];
			}

			if (pre.second == 0) {
				ask(1, 0);
			} else if (pre.second == 1) {
				ask(2, 1);
			} else {
				ask(2, 0);
			}

		}

	}
	cout << "4 " << ans_cost << endl;


	return 0;
}
