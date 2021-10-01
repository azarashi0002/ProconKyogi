#include<Siv3D.hpp>
#include<Eigen/Core>
using namespace Eigen;
using namespace std;

using TPoint = Point;
using GPoint = Point;

Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> ImageToMatrix(const Image& image) {
	Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> ret(image.width(), image.height());
	for (const auto [x, y] : step(image.size())) {
		const auto [r, g, b, a] = ColorF(image[y][x]);
		ret(y, x) << r, g, b;
	}
	return ret;
}
std::array<Eigen::MatrixX3d, 4> PickEdge(const Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic>& panel) {
	std::array<Eigen::MatrixX3d, 4> ret;
	std::array<Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, 1>, 4> edge = {
		panel.topRows(1).row(0).transpose().eval(),
		panel.rightCols(1).col(0).eval(),
		panel.bottomRows(1).row(0).transpose().reverse().eval(),
		panel.leftCols(1).col(0).reverse().eval()
	};

	for (int i : step(4)) {
		Eigen::MatrixX3d mat(panel.rows(), 3);
		for (int j : step(panel.rows())) {
			mat.row(j) = edge[i][j];
		}
		ret[i] = mat;
	}
	return ret;
}
pair<GPoint, int> GetP2PosAndRotate(const GPoint& p1, int p1r, int p1e, int p2e) {
	return make_pair<GPoint, int>(p1 + s3d::Array<Point>{ Point::Up(), Point::Right(), Point::Down(), Point::Left() } [(p1r + p1e) % 4] , (p1r + p1e + p2e + 2) % 4);
}

class PPMInfo {
public:
	PPMInfo(StringView filepath) :
		m_reader(filepath) {
		if (not m_reader)return;
		m_reader.readLine();
		auto t = m_reader.readLine().value().split(U' ');
		divit.x = ParseInt<int>(t[1]);
		divit.y = ParseInt<int>(t[2]);
		t = m_reader.readLine().value().split(U' ');
		select_num = ParseInt<int>(t[1]);
		t = m_reader.readLine().value().split(U' ');
		cost.select = ParseInt<int>(t[1]);
		cost.swap = ParseInt<int>(t[2]);
	}
private:
	TextReader m_reader;
public:
	Size divit;
	int select_num;
	struct Cost {
		int select;
		int swap;
	}cost;
};

struct GraphData {
	TPoint p;
	int r;
};

void Main()
{
	Window::SetStyle(WindowStyle::Sizable);
	Scene::SetBackground(Scene::DefaultBackgroundColor);

	constexpr StringView path = U"testcase/problem2.ppm";

	const Image image = PPMDecoder().decode(path);
	const Texture texture(image);
	const PPMInfo info(path);
	const Size tile_size = image.size() / info.divit;
	const Grid<Image> tiles = Grid<Image>::IndexedGenerate(info.divit, [&](const Point& p) {return image.clipped(Rect(p * tile_size, tile_size)); });
	const auto mat_tiles = (tiles >> [](const auto& image) {return ImageToMatrix(image); });
	const Grid<std::array<Eigen::MatrixX3d, 4>> edge = (mat_tiles >> [](const auto& mat) {return PickEdge(mat); });

	struct ScoreData {
		Point p1, p2;
		int e1, e2;
		double score;
	};
	const auto score_calc = [&](const TPoint& p1, int e1, const TPoint& p2, int e2) {
		return (edge[p1][e1] - edge[p2][e2].colwise().reverse()).rowwise().squaredNorm().sum();
	};

	HashTable<TPoint, HashTable<TPoint, HashTable<int, HashTable<int, double>>>> scoretable;
	s3d::Array<ScoreData> scorearray;
	for (auto i : step(tiles.size())) {
		for (auto j : step(tiles.size())) {
			for (auto k : step(4)) {
				for (int l : step(4)) {
					const double score = score_calc(i, k, j, l);
					scoretable[i][j][k][l] = score;
					scorearray << ScoreData{ i,j,k,l,score };
				}
			}
		}
	}

	Grid<Optional<GraphData>> graph(64, 64);
	graph[{32, 32}] = GraphData{ {0,0},0 };
	const auto updategraph = [&] {
		s3d::Array<TPoint> used;
		for (const auto& i : graph) {
			if (not i)continue;
			used << i->p;
		}
		Logger << U"used.size()={}"_fmt(used.size());
		Logger << used;

		s3d::Array<ScoreData> data;
		for (const auto& i : used) {
			data.append(scorearray.removed_if([&](const ScoreData& d) {return not(d.p1 == i or d.p2 == i); }));
		}
		Logger << U"1:data.size()={}"_fmt(data.size());
		data.remove_if([&](const ScoreData& d) {return used.includes(d.p1) and used.includes(d.p2); });
		Logger << U"2:data.size()={}"_fmt(data.size());
		if (data.empty()) {
			Logger << U"data.empty()";
			return false;
		}
		data.sort_by([](const ScoreData& d1, const ScoreData& d2) {return d1.score < d2.score; });

		while (true) {
			const ScoreData& minscore = data.front();
			const TPoint own = used.includes(minscore.p1) ? minscore.p1 : minscore.p2;
			const TPoint target = used.includes(minscore.p1) ? minscore.p2 : minscore.p1;
			Logger << U"own={},target={}"_fmt(own, target);
			const int owne = used.includes(minscore.p1) ? minscore.e1 : minscore.e2;
			const int targete = used.includes(minscore.p1) ? minscore.e2 : minscore.e1;
			GPoint owngp(-1, -1);
			for (GPoint p : step(graph.size())) {
				if (graph[p] and graph[p]->p == own) {
					owngp = p;
					break;
				}
			}
			const auto [gp, r] = GetP2PosAndRotate(owngp, graph[owngp]->r, owne, targete);
			if (graph[gp]) {
				data.pop_front();
				Logger << U"continue";
				continue;
			}
			graph[gp] = GraphData{ target,r };
			break;
		}
		return true;
	};

	s3d::Console << U"Step1";
	for (auto n : step(tiles.num_elements() - 1)) {
		if (not updategraph()) {
			break;
		}
	}

	Rect maxtile_region(0, 0, tiles.size());
	int tilecount = 0;
	for (auto i : step(Size(64, 64) - tiles.size() + Point(1, 1))) {
		int count = 0;
		for (auto j : step(i, tiles.size())) {
			if (graph[j])count++;
		}

		if (tilecount < count) {
			tilecount = count;
			maxtile_region.pos = i;
		}
	}
	for (auto i : step(graph.size())) {
		if (not maxtile_region.intersects(i)) {
			graph[i].reset();
		}
	}

	s3d::Console << maxtile_region;

	const auto updategraph2 = [&] {
		s3d::Array<TPoint> used;
		for (const auto& i : graph) {
			if (not i)continue;
			used << i->p;
		}
		Logger << U"used.size()={}"_fmt(used.size());
		Logger << used;

		s3d::Array<ScoreData> data;
		for (const auto& i : used) {
			data.append(scorearray.removed_if([&](const ScoreData& d) {return not(d.p1 == i or d.p2 == i); }));
		}
		Logger << U"1:data.size()={}"_fmt(data.size());
		data.remove_if([&](const ScoreData& d) {return used.includes(d.p1) and used.includes(d.p2); });
		Logger << U"2:data.size()={}"_fmt(data.size());
		if (data.empty()) {
			Logger << U"data.empty()";
			return false;
		}
		data.sort_by([](const ScoreData& d1, const ScoreData& d2) {return d1.score < d2.score; });

		while (true) {
			const ScoreData& minscore = data.front();
			const TPoint own = used.includes(minscore.p1) ? minscore.p1 : minscore.p2;
			const TPoint target = used.includes(minscore.p1) ? minscore.p2 : minscore.p1;
			Logger << U"own={},target={}"_fmt(own, target);
			const int owne = used.includes(minscore.p1) ? minscore.e1 : minscore.e2;
			const int targete = used.includes(minscore.p1) ? minscore.e2 : minscore.e1;
			GPoint owngp(-1, -1);
			for (GPoint p : step(graph.size())) {
				if (graph[p] and graph[p]->p == own) {
					owngp = p;
					break;
				}
			}
			const auto [gp, r] = GetP2PosAndRotate(owngp, graph[owngp]->r, owne, targete);
			if (graph[gp] or not maxtile_region.intersects(gp)) {
				data.pop_front();
				Logger << U"continue";
				continue;
			}
			graph[gp] = GraphData{ target,r };
			break;
		}
		return true;
	};

	s3d::Console << U"Step2";
	for (auto n : step(tiles.num_elements() - 1)) {
		if (not updategraph2()) {
			break;
		}
	}

	Grid<GraphData> graph2(tiles.size());
	for (auto i : step(graph2.size())) {
		graph2[i] = *graph[maxtile_region.pos + i];
	}

	{
		TextWriter writer(U"PLANA.txt", TextEncoding::UTF8_NO_BOM);
		writer << info.divit.y << U' ' << info.divit.x;
		writer << info.cost.select << U' ' << info.cost.swap;
		for (const auto i : step(tiles.height())) {
			String s;
			for (const auto j : step(tiles.width())) {
				for (const auto k : step(tiles.size())) {
					if (graph2[k].p == Point(j, i)) {
						s += U"{} {} {} "_fmt(k.y, k.x, graph2[k].r);
					}
				}
			}
			writer << s;
		}
	}

	Image im(tile_size * graph2.size());
	DynamicTexture t(im);
	while (System::Update())
	{
		const auto rotate = [](const Image& image, int r) {
			switch (r) {
			case 0:
				return image;
			case 1:
				return image.rotated90();
			case 2:
				return image.rotated180();
			case 3:
				return image.rotated270();
			}
		};
		for (const auto& i : step(graph2.size())) {
			rotate(tiles[graph2[i].p], graph2[i].r).overwrite(im, tile_size * i);
		}
		t.fill(im);
		t.scaled(0.5).draw();
	}
}
