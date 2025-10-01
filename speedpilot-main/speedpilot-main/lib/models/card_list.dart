class CardList {
  final String name;
  final String ipaddress;

  // Constructor to initialize name and IP address
  CardList(
    this.name,
    this.ipaddress,
  );

  // Convert object to JSON map
  Map<String, String> toJson() => {
        'name': name,
        'ipaddress': ipaddress,
      };

  // Create a CardList instance from a JSON map
  factory CardList.fromJson(Map<String, dynamic> json) {
    return CardList(
      json['name'] ?? '',
      json['ipaddress'] ?? '',
    );
  }
}
