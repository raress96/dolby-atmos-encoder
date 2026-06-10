// cavernprobe — verify a DD+/E-AC-3 (or MKV/WAV) stream with VoidXH/Cavern: report channel layout,
// whether Cavern sees Dolby Atmos *objects*, and the decoder's full metadata (object_count,
// joc_num_objects, …). Exit code 0 = objects present, 2 = channel-based only, 64 = bad usage.
//
// Build:  dotnet build tools/cavernprobe -c Release
// Run:    dotnet tools/cavernprobe/bin/Release/net8.0/cavernprobe.dll <file.eac3|.mkv|.wav>

using System;
using Cavern.Format;
using Cavern.Format.Common;

if (args.Length < 1)
{
    Console.Error.WriteLine("usage: cavernprobe <file.eac3|.ec3|.ac3|.mkv|.wav>");
    return 64;
}

string path = args[0];
AudioReader reader = AudioReader.Open(path);
reader.ReadHeader();

Console.WriteLine($"file        : {path}");
Console.WriteLine($"channels    : {reader.ChannelCount}");
Console.WriteLine($"sample rate : {reader.SampleRate} Hz");
Console.WriteLine($"length      : {reader.Length} samples");

bool hasObjects = reader.GetRenderer().HasObjects;
Console.WriteLine($"HasObjects  : {hasObjects}");

if (reader is IMetadataSupplier supplier)
{
    ReadableMetadata md = supplier.GetMetadata();
    foreach (ReadableMetadataHeader header in md.Headers)
    {
        Console.WriteLine($"[{header.Name}]");
        foreach (ReadableMetadataField field in header.Fields)
        {
            Console.WriteLine($"  {field}");
        }
    }
}

Console.WriteLine(hasObjects
    ? "RESULT: object-based audio detected (Atmos objects present)."
    : "RESULT: no objects — channel-based only.");
return hasObjects ? 0 : 2;
