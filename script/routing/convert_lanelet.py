from pylanelet import Lanelet2Converter

if __name__ == "__main__":
    input_file = "../../cfg/track.osm"
    output_file = "../../cfg/track.pkl"
    converter = Lanelet2Converter(input_file)
    converter.convert_lanelet2()
    converter.save_map(output_file)