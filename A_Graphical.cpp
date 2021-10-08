#include<Siv3D.hpp>
#include<Eigen/Core>

using ImageMatrix = Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic>;
using ScoreTable = HashTable<Point, HashTable<Point, HashTable<int, HashTable<int, double>>>>;	//table[p1][p2][e1][e2]

class Tile;
class Edge {
public:
	Eigen::MatrixX3d value;

	struct ParentData {
		Tile* ptr = nullptr;
		int edge;	//0~3
	}parent;
	struct Score {
		const Edge* target = nullptr;
		double score;
	};
	Array<Score> scores;
};
using Edges = std::array<Edge, 4>;

class Node {
public:
	Node(const Rect& region, const Point& index) :
		region(region),
		index(index) {}
	Tile* on = nullptr;
	Rect region;

	Point index;
};
using Graph = Grid<Node>;

ImageMatrix ImageToMatrix(const Image& image) {
	Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> ret(image.width(), image.height());
	for (const auto [x, y] : step(image.size())) {
		const auto [r, g, b, a] = ColorF(image[y][x]);
		ret(y, x) << r, g, b;
	}
	return ret;
}
template<class Derived>
Edges PickEdges(const Eigen::MatrixBase<Derived>& panel) {
	Edges ret;
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
		ret[i] = Edge{ mat };
	}
	return ret;
}

ScoreTable MakeScoreTable(const Grid<Edges>& edges) {
	ScoreTable table;
	const auto score_calc = [&](const Point& p1, int e1, const Point& p2, int e2) {
		return (edges[p1][e1].value - edges[p2][e2].value.colwise().reverse()).rowwise().squaredNorm().sum();
	};

	for (auto i : step(edges.size())) {
		Logger << U"MakeScoreTable::i={}"_fmt(i);
		for (auto j : step(edges.size())) {
			for (auto k : step(4)) {
				for (int l : step(4)) {
					table[i][j][k][l] = score_calc(i, k, j, l);
				}
			}
		}
	}

	return table;
}

class PPMInfo {
public:
	PPMInfo(FilePathView filepath) :
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

class Tile {
public:
	Tile(const Image& image, const Rect& region) :
		image(image),
		texture(image),
		matrix(ImageToMatrix(image)),
		edges(PickEdges(matrix)),
		region(region) {}
	void draw() const {
		texture.resized(region.size).rotated(rotate * Math::HalfPi).draw(region.pos);
	}
	bool mouseOver() const {
		return region.mouseOver();
	}
	bool leftClicked() const {
		return region.leftClicked();
	}
public:
	Image image;
	Texture texture;
	ImageMatrix matrix;
	Edges edges;
	Rect region;

	int rotate = 0;

	Node* base = nullptr;
};

void Main() {
	Window::SetStyle(WindowStyle::Sizable);

	constexpr FilePathView path = U"testcase/problem2.ppm";
	const PPMDecoder decoder;
	const PPMInfo info(path);


	Logger << U"begin initialize";
	Logger << U"decode {}"_fmt(path);
	const Image image = decoder.decode(path);
	Logger << U"grid image";
	const auto images = Grid<Image>::IndexedGenerate(info.divit, [&](const Point& p) {return image.clipped(Rect(image.size() / info.divit * p, image.size() / info.divit)); });
	Logger << U"cast to struct Tile";
	auto tiles = Grid<Tile>::IndexedGenerate(images.size(), [&](const Point& p) {return Tile(images[p], Rect(image.size() / info.divit * p, image.size() / info.divit)); });
	Logger << U"set edges parent";
	for (auto&& i : tiles) {
		for (int j : step(i.edges.size())) {
			i.edges[j].parent = Edge::ParentData{ &i,j };
		}
	}
	Logger << U"make score table";
	const auto score_table = MakeScoreTable(tiles >> [](const Tile& tile) {return tile.edges; });
	Logger << U"set edges connection";
	for (const auto& [p1, v1] : score_table) {
		for (const auto& [p2, v2] : v1) {
			for (const auto& [e1, v3] : v2) {
				for (const auto& [e2, score] : v3) {
					if (p1 == p2) {
						continue;
					}
					tiles[p1].edges[e1].scores.push_back(Edge::Score{ &tiles[p2].edges[e2],score_table.at(p1).at(p2).at(e1).at(e2) });
				}
			}
		}
	}
	Logger << U"sort all edge's score table";
	for (auto&& i : tiles) {
		for (auto&& j : i.edges) {
			j.scores.sort_by([](const Edge::Score& s1, const Edge::Score& s2) {return s1.score < s2.score; });
		}
	}
	Logger << U"end initialize";

	Point gridpos = { image.width() + 100,0 };

	auto base = Grid<Node>::IndexedGenerate(tiles.size(), [&](const Point& i) {return Node(Rect(i * image.size() / info.divit + gridpos, image.size() / info.divit), i); });

	Texture texture(image);


	const Tile* mouseOverTile = nullptr;
	Array<Tile*> grabTiles;

	int rank = 3;
	int thickness = 3;

	Optional<Point> selectRegionClicked, selectRegionReleased;

	while (System::Update()) {

		{
			Transformer2D t(Mat3x2::Scale(0.4), TransformCursor::Yes);

			mouseOverTile = nullptr;

			//update
			{
				//bef
				{
					if (not grabTiles) {
						if (MouseR.down()) {
							selectRegionClicked = Cursor::Pos();
							selectRegionReleased = none;
						}
						if (MouseR.up() && selectRegionClicked) {
							selectRegionReleased = Cursor::Pos();

							//小さすぎないかのチェック
							const Point p1 = { Min(selectRegionClicked->x,selectRegionReleased->x),Min(selectRegionClicked->y,selectRegionReleased->y) };
							const Point p2 = { Max(selectRegionClicked->x,selectRegionReleased->x),Max(selectRegionClicked->y,selectRegionReleased->y) };
							if (p1.distanceFromSq(p2) < 400) {
								selectRegionClicked = none;
								selectRegionReleased = none;
							}
							else {
								const Rect selectRegion(p1, p2 - p1);
								for (auto&& i : tiles) {
									if (i.region.intersects(selectRegion)) {
										grabTiles << &i;
									}
								}
							}
						}
					}

					if (grabTiles) {
					}
					else {
						for (auto&& i : tiles) {
							if (i.mouseOver()) {
								mouseOverTile = &i;
								break;
							}
						}
					}
					if (mouseOverTile) {
						Cursor::RequestStyle(CursorStyle::Hand);
					}

					if (MouseL.up() && not (selectRegionClicked && selectRegionReleased)) {
						grabTiles.clear();
					}
					if (not (selectRegionClicked && selectRegionReleased)) {
						for (auto&& i : tiles) {
							if (i.leftClicked()) {
								grabTiles << &i;
								selectRegionClicked = selectRegionReleased = none;
								break;
							}
						}
					}
					if (grabTiles) {
						if (selectRegionClicked && selectRegionReleased) {
							const Point p1 = { Min(selectRegionClicked->x,selectRegionReleased->x),Min(selectRegionClicked->y,selectRegionReleased->y) };
							const Point p2 = { Max(selectRegionClicked->x,selectRegionReleased->x),Max(selectRegionClicked->y,selectRegionReleased->y) };
							const Rect selectRegion(p1, p2 - p1);
							if ((MouseL | MouseR).down() && not selectRegion.mouseOver()) {
								for (auto&& i : grabTiles) {
									for (auto&& j : base) {
										if (not j.on && j.region.intersects(i->region.center())) {
											i->region.setPos(j.region.pos);
											j.on = i;
											i->base = &j;
										}
									}
								}

								selectRegionClicked = selectRegionReleased = none;
								grabTiles.clear();
							}
							else if (MouseL.pressed()) {
								Cursor::RequestStyle(CursorStyle::Hidden);
								for (auto&& i : grabTiles) {
									i->region.moveBy(Cursor::Delta());
									if (i->base) {
										i->base->on = nullptr;
										i->base = nullptr;
									}
								}
								selectRegionClicked->moveBy(Cursor::Delta());
								selectRegionReleased->moveBy(Cursor::Delta());
							}
						}
						else {
							Cursor::RequestStyle(CursorStyle::Hidden);
							if (Rect(base.asArray().front().region.tl(), base.asArray().back().region.br() - base.asArray().front().region.tl()).mouseOver()) {	//baseのグリッド全体
								for (auto&& i : base) {
									if ((not i.on or i.on == grabTiles[0]) && i.region.mouseOver()) {
										if (grabTiles[0]->base) {
											grabTiles[0]->base->on = nullptr;
											grabTiles[0]->base = nullptr;
										}
										grabTiles[0]->region.setPos(i.region.pos);
										grabTiles[0]->base = &i;
										i.on = grabTiles[0];
									}
								}
							}
							else {
								grabTiles[0]->region.setCenter(Cursor::Pos());
								if (grabTiles[0]->base) {
									grabTiles[0]->base->on = nullptr;
									grabTiles[0]->base = nullptr;
								}
							}
						}

						if (MouseR.down()) {
							for (auto&& i : grabTiles) {
								i->rotate++;
								i->rotate %= 4;
							}
						}
					}
				}
			}

			//draw
			{
				{
					for (const auto& i : base) {
						if (i.on) {
							Circle(i.region.center(), 3.0).draw(Palette::Red);

							if (i.on->base && i.on->base == &i) {
								Line(i.region.center(), i.on->region.center()).draw(1.0, Palette::Red);
							}
						}
					}
				}

				//aft
				{
					for (const auto& i : base) {
						i.region.drawFrame(1.0, 0.0, Palette::White);
					}
				}
				//bef
				{
					for (const auto& i : tiles) {
						i.draw();
					}
					if (mouseOverTile) {
						constexpr std::array<Color, 4> colors = { Palette::Darkred, Palette::Darkgreen, Palette::Darkblue, Palette::Darkorange };

						for (const auto& [i, v] : Indexed(mouseOverTile->edges)) {
							for (const auto& [j, v2] : Indexed(v.scores.take(rank))) {	//上位3つ
								const Edge::ParentData& parent = v2.target->parent;
								const std::array<Line, 4> parentedges = { parent.ptr->region.top(),parent.ptr->region.right(),parent.ptr->region.bottom(),parent.ptr->region.left() };

								parentedges[(parent.edge + parent.ptr->rotate) % 4].draw(thickness * (rank - j - 1) + 1, colors[i % 4]);
								parent.ptr->region.draw(Palette::White.withAlpha(Periodic::Sine0_1(1.5s) * 25 + 25));
							}
						}

						mouseOverTile->region.top().draw(3.0, colors[(4 - mouseOverTile->rotate) % 4]);
						mouseOverTile->region.right().draw(3.0, colors[(5 - mouseOverTile->rotate) % 4]);
						mouseOverTile->region.bottom().draw(3.0, colors[(6 - mouseOverTile->rotate) % 4]);
						mouseOverTile->region.left().draw(3.0, colors[(7 - mouseOverTile->rotate) % 4]);
					}
				}

				//selectarea
				{
					if (not grabTiles && selectRegionClicked && MouseR.pressed()) {
						const Point p1 = { Min(selectRegionClicked->x,Cursor::Pos().x),Min(selectRegionClicked->y,Cursor::Pos().y) };
						const Point p2 = { Max(selectRegionClicked->x,Cursor::Pos().x),Max(selectRegionClicked->y,Cursor::Pos().y) };
						const Rect selectRegion(p1, p2 - p1);

						selectRegion.draw(Palette::Pink.withAlpha(64));
					}
					else if (selectRegionClicked && selectRegionReleased) {
						const Point p1 = { Min(selectRegionClicked->x,selectRegionReleased->x),Min(selectRegionClicked->y,selectRegionReleased->y) };
						const Point p2 = { Max(selectRegionClicked->x,selectRegionReleased->x),Max(selectRegionClicked->y,selectRegionReleased->y) };
						const Rect selectRegion(p1, p2 - p1);

						selectRegion.draw(Palette::Pink.withAlpha(64));
					}
				}
			}
		}

		//GUI
		{
			if (KeyR.pressed()) {
				rank = Clamp(rank + int(Mouse::Wheel()), 0, 10);
			}
			{
				double t = rank;
				SimpleGUI::Slider(U"rank(R)={}"_fmt(rank), t, 0, 10, { 0,600 }, 200, 100);
				rank = int(t);
			}

			if (KeyT.pressed()) {
				thickness = Clamp(thickness + int(Mouse::Wheel()), 0, 10);
			}
			{
				double t = thickness;
				SimpleGUI::Slider(U"thickness(T)={}"_fmt(thickness), t, 0, 10, { 0,636 }, 200, 100);
				thickness = int(t);
			}
			{
				auto output = [&] {
					TextWriter writer(U"PLANA.txt", TextEncoding::UTF8_NO_BOM);
					writer << info.divit.y << U' ' << info.divit.x;
					writer << info.cost.select << U' ' << info.cost.swap;
					for (auto i : step(tiles.height())) {
						String s;
						for (auto j : step(tiles.width())) {
							s += U"{} {} {} "_fmt(tiles[i][j].base->index.y, tiles[i][j].base->index.x, tiles[i][j].rotate);
						}
						writer << s;
					}
				};
				if (KeyO.pressed()) {
					if (base.all([](const Node& n) {return n.on; })) {
						output();
					}
				}
				if (SimpleGUI::Button(U"output Textfile(O)", { 0,672 }, unspecified, base.all([](const Node& n) {return n.on; }))) {
					output();
				}
			}

			if (KeyEnter.down() || KeyShift.pressed()) {
				if (mouseOverTile) {
					for (const auto& [i, v] : Indexed(mouseOverTile->edges)) {
						constexpr std::array<Point, 4> p = { Point::Up(),Point::Right(),Point::Down(),Point::Left() };
						if (const auto& ii = mouseOverTile->base->index + p[(i + mouseOverTile->rotate) % 4]; base.inBounds(ii)) {
							if (auto&& ref = base[ii]; not ref.on) {
								auto& t = v.scores.removed_if([&](const Edge::Score& s) {return base.includes_if([&](const Node& n) {return n.on == s.target->parent.ptr; }); }).front().target->parent;
								ref.on = t.ptr;
								ref.on->region.setPos(ref.region.pos);
								ref.on->rotate = (6 + i + mouseOverTile->rotate - t.edge) % 4;
								ref.on->base = &ref;
							}
						}
					}
				}
				else {
				}
			}
		}
	}
}
